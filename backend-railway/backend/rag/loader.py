from typing import List
import os
import re
from markdown_it import MarkdownIt

class Document:
    def __init__(self, text: str, metadata: dict):
        self.text = text
        self.metadata = metadata

def _strip_markdown(content: str) -> str:
    md = MarkdownIt()
    html = md.render(content)
    # Simple regex to strip HTML tags for now, can be improved
    text = re.sub(r'<[^>]+>', '', html)
    # Further cleanup: remove excessive newlines, decode HTML entities
    text = re.sub(r'\n\s*\n', '\n', text).strip()
    return text

def load_and_clean_documents(docs_path: str) -> List[Document]:
    documents = []
    for root, _, files in os.walk(docs_path):
        for file_name in files:
            if file_name.endswith((".md", ".mdx")):
                file_path = os.path.join(root, file_name)
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                
                cleaned_text = _strip_markdown(content)
                metadata = {
                    "source": file_path,
                    "filename": file_name,
                    # Add chapter/section parsing logic here if needed
                }
                documents.append(Document(text=cleaned_text, metadata=metadata))
    return documents
