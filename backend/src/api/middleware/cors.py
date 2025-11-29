from fastapi.middleware.cors import CORSMiddleware
from ...core.config import settings

def setup_cors_middleware(app):
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:3000", "https://<YOUR_GITHUB_PAGES_DOMAIN>"],  # TODO: Configure dynamic origins from settings
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )