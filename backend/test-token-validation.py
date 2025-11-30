#!/usr/bin/env python3
"""Test token validation with Better Auth"""
import asyncio
import httpx

async def test_token_validation():
    # Token from the signup response (unsigned)
    unsigned_token = "DyNYWhG5aCaWYZm17To2GqaKknlkuoIS"
    
    # Try with unsigned token
    print("Testing with UNSIGNED token...")
    async with httpx.AsyncClient() as client:
        response = await client.get(
            "http://localhost:3001/api/auth/get-session",
            headers={"Cookie": f"better-auth.session_token={unsigned_token}"},
            timeout=5.0
        )
        print(f"Status: {response.status_code}")
        print(f"Response: {response.text[:200]}")
        print()

if __name__ == "__main__":
    asyncio.run(test_token_validation())
