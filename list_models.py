import google.generativeai as genai
import os

# 使用您的 API Key
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "AIzaSyDqT-0MfSvqSYCqw4RrRKE5j9GIHCGvLG0")
genai.configure(api_key=GEMINI_API_KEY)

print("Listing available models...")
try:
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(f"- {m.name}")
except Exception as e:
    print(f"Error: {e}")
