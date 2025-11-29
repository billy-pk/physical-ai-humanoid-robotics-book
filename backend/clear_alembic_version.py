#!/usr/bin/env python3
"""Clear old Alembic version from database."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from src.core.config import settings
import psycopg

print("Clearing old Alembic version...")

with psycopg.connect(settings.NEON_DATABASE_URL) as conn:
    with conn.cursor() as cur:
        # Delete old alembic version
        cur.execute("DELETE FROM alembic_version;")
        conn.commit()
        print("✅ Cleared old Alembic version")

print("Now run: uv run alembic upgrade head")
