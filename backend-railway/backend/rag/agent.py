import os
from openai import OpenAI
from dotenv import load_dotenv
from typing import List, Dict, Any

load_dotenv()

# Initialize OpenRouter client, but handle potential missing API key
try:
    api_key = os.environ.get("OPENROUTER_API_KEY")
    if not api_key:
        print("Warning: OPENROUTER_API_KEY not found. LLM will be disabled.")
        llm_client = None
    else:
        llm_client = OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        LLM_MODEL = "mistralai/devstral-2512:free"
except Exception as e:
    print(f"Error initializing OpenRouter client: {e}. LLM will be disabled.")
    llm_client = None

# Refusal messages
REFUSAL_EN = "I'm sorry, I can only answer questions related to the textbook content."
REFUSAL_UR = "Maaf kijiye, mein sirf textbook ke content se related sawalon ka jawab de sakta hoon."

def _format_rag_only_response(context_chunks: List[str], source_references: List[str]) -> Dict[str, Any]:
    """Formats the RAG context into a user-friendly response when the LLM fails."""
    if not context_chunks:
        return {
            "answer": "No relevant context found in the book.",
            "detailed_answer": "No relevant context was found to answer your question. Please try rephrasing or selecting different text.",
            "sources": []
        }
        
    # Combine and format the retrieved chunks as the main answer
    formatted_answer = "Here is the most relevant information found in the textbook:\n\n" + "\n\n---\n\n".join(context_chunks)
    
    unique_sources = sorted(list(set(source_references)))

    return {
        "answer": "Could not generate a summary, but here is the relevant context from the book.",
        "detailed_answer": formatted_answer,
        "sources": unique_sources
    }

def detect_language(text: str) -> str:
    """Detects if the text is English or Roman Urdu using the OpenRouter model. Falls back to English on error."""
    if not llm_client:
        return "en"
    try:
        prompt = f"Identify the language of the following text: '{text}'. Respond with 'en' for English or 'ur' for Roman Urdu."
        response = llm_client.chat.completions.create(
            model=LLM_MODEL,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=5,
        )
        lang_code = response.choices[0].message.content.strip().lower()
        return "ur" if "ur" in lang_code else "en"
    except Exception as e:
        print(f"Error detecting language, defaulting to 'en': {e}")
        return "en"

def check_relevance(question: str) -> bool:
    """Checks if the question is relevant. Defaults to True on error to avoid blocking valid questions."""
    if not llm_client:
        return True
    try:
        prompt = f"Is the following question relevant to a 'Humanoid Robotics' textbook? Answer with 'yes' or 'no'. Question: '{question}'"
        response = llm_client.chat.completions.create(
            model=LLM_MODEL,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=3,
        )
        return "yes" in response.choices[0].message.content.strip().lower()
    except Exception as e:
        print(f"Error checking relevance, defaulting to 'True': {e}")
        return True

def generate_rag_response(question: str, context_chunks: List[str], source_references: List[str]) -> Dict[str, Any]:
    """
    Generates a response using RAG, with an optional LLM fallback.
    Ensures that any failure in LLM communication is caught and handled gracefully.
    """
    if not context_chunks:
        # If no context is found from RAG, return a "not found" message immediately.
        return {
            "answer": "Answer not found in the book. Please try rephrasing your question.",
            "detailed_answer": "No relevant content was found in the textbook to answer your question.",
            "sources": []
        }

    # If the LLM is disabled via configuration, go directly to the RAG-only response.
    if not llm_client:
        print("LLM disabled. Falling back to RAG-only response.")
        return _format_rag_only_response(context_chunks, source_references)

    try:
        # --- Start of fully isolated LLM logic ---
        # All calls that depend on the LLM are now inside this single try block.

        lang = detect_language(question)
        is_relevant = check_relevance(question)

        if not is_relevant:
            refusal_message = REFUSAL_EN if lang == "en" else REFUSAL_UR
            return {"answer": refusal_message, "detailed_answer": refusal_message, "sources": []}

        # Format the prompt for the LLM
        formatted_context = "\n\n".join(context_chunks)
        system_message = (
            "You are a helpful assistant for a Humanoid Robotics textbook. Answer questions strictly from the provided context. "
            "If the answer is not in the context, say 'Answer not found in the book.' "
            "Provide a concise summary first, then a detailed answer with citations. "
            f"Respond in {'Roman Urdu' if lang == 'ur' else 'English'}."
        )
        user_message = f"Context from the book:\n{formatted_context}\n\nQuestion: {question}"
        
        response = llm_client.chat.completions.create(
            model=LLM_MODEL,
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_message}
            ],
            temperature=0.2,
        )

        llm_full_response = response.choices[0].message.content.strip()
        
        # Process and return the successful LLM response
        concise_answer = llm_full_response.split('\n\n')[0]
        detailed_answer = llm_full_response
        unique_sources = sorted(list(set(source_references)))

        return {
            "answer": concise_answer,
            "detailed_answer": detailed_answer,
            "sources": unique_sources
        }
        # --- End of fully isolated LLM logic ---
        
    except Exception as e:
        # If any part of the LLM logic fails (network, API, etc.), catch it.
        print(f"Error generating LLM response: {e}. Falling back to RAG-only response.")
        # Fallback to returning the raw context if the LLM fails for any reason.
        return _format_rag_only_response(context_chunks, source_references)