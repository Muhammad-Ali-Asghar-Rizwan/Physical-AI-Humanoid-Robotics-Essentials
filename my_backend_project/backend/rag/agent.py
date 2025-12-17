
import os
import sys
import asyncio
import google.generativeai as genai
from dotenv import load_dotenv
from typing import List, Dict, Any

# Add the backend directory to the path to enable imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

load_dotenv()
genai.configure(api_key=os.environ.get("GEMINI_API_KEY"))

model_rag = genai.GenerativeModel('gemini-2.0-flash')

# Refusal messages
REFUSAL_EN = "I'm sorry, I can only answer questions related to the textbook content."
REFUSAL_UR = "Maaf kijiye, mein sirf textbook ke content se related sawalon ka jawab de sakta hoon."

async def detect_language(text: str) -> str:
    try:
        prompt = f"Identify the language of the following text: '{text}'. Respond with 'en' for English or 'ur' for Roman Urdu."
        response = await asyncio.wait_for(model_rag.generate_content(prompt), timeout=3.0)
        lang_code = response.text.strip().lower()
        if "ur" in lang_code:
            return "ur"
        return "en"
    except Exception as e:
        print(f"Error detecting language: {e}")
        return "en"

async def check_relevance(question: str) -> bool:
    try:
        prompt = f"Is the following question relevant to a 'Humanoid Robotics' textbook? Answer with 'yes' or 'no'. Question: '{question}'"
        response = await asyncio.wait_for(model_rag.generate_content(prompt), timeout=3.0)
        relevance_check = response.text.strip().lower()
        return "yes" in relevance_check
    except Exception as e:
        print(f"Error checking relevance: {e}")
        return True


async def generate_rag_response(question: str, context_chunks: List[str], source_references: List[str]) -> Dict[str, Any]:
    lang = await detect_language(question)

    if not context_chunks:
        # Check relevance even if context is empty, to provide a relevant refusal
        is_relevant = await check_relevance(question)
        if not is_relevant:
            refusal_message = REFUSAL_EN if lang == "en" else REFUSAL_UR
            return {
                "answer": refusal_message,
                "detailed_answer": refusal_message,
                "sources": []
            }

        return {
            "answer": "Answer not found in the book. Please try rephrasing your question or selecting more relevant text.",
            "detailed_answer": "Answer not found in the book. Please try rephrasing your question or selecting more relevant text.",
            "sources": []
        }

    # Check relevance for non-empty context too
    is_relevant = await check_relevance(question)
    if not is_relevant:
        refusal_message = REFUSAL_EN if lang == "en" else REFUSAL_UR
        return {
            "answer": refusal_message,
            "detailed_answer": refusal_message,
            "sources": []
        }

    formatted_context = "\n\n".join(context_chunks)

    # System rules: Answer ONLY from provided context. If answer not found -> say "Answer not found in the book".
    # Provide a concise summary first, and then elaborate with more detail in a separate 'detailed_answer' section, including specific citations.
    system_message = (
        "You are a helpful assistant for a Humanoid Robotics textbook. Your primary goal is to answer questions strictly from the provided context from the book. "
        "If the answer is not found within the given context, clearly state 'Answer not found in the book.' Do not make up answers. "
        "Provide a concise summary first, and then elaborate with more detail in a separate 'detailed_answer' section, including specific citations to the provided sources (e.g., 'Source: chapter1.md'). "
        f"Respond in {{'Roman Urdu' if lang == 'ur' else 'English'}}."
    )
    user_message = f"Context from the book:\n{formatted_context}\n\nQuestion: {question}"

    full_prompt = f"{system_message}\n\n{user_message}"

    try:
        response = await asyncio.wait_for(model_rag.generate_content(
            full_prompt,
            generation_config={"temperature": 0.2}
        ), timeout=12.0)
        llm_full_response = response.text.strip()

        concise_answer = llm_full_response.split('\n\n')[0] # First paragraph as concise
        detailed_answer = llm_full_response

        unique_sources = list(set(source_references))

        return {
            "answer": concise_answer,
            "detailed_answer": detailed_answer,
            "sources": unique_sources
        }
    except Exception as e:
        print(f"Error generating LLM response: {e}")
        # Graceful error message based on detected language
        graceful_error_en = "I’m having trouble generating a response right now. Please try again."
        graceful_error_ur = "Abhi jawab generate karne mein masla aa raha hai, thori dair baad dobara koshish karein."

        error_message = graceful_error_en if lang == "en" else graceful_error_ur

        return {
            "answer": error_message,
            "detailed_answer": error_message,
            "sources": []
        }
