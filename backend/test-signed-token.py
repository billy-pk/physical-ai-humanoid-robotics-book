#!/usr/bin/env python3
"""Test with the actual signed cookie"""
import asyncio
import httpx

async def test_signed_token():
    # This is the SIGNED token from the cookie (includes signature after the dot)
    signed_token = "DyNYWhG5aCaWYZm17To2GqaKknlkuoIS.nD9N+u8NWJ6Dv9gW5YuNJgBlCw9rY47b+kYYKJ5Lwok="
    
    print("Testing with SIGNED cookie token...")
    async with httpx.AsyncClient() as client:
        response = await client.get(
            "http://localhost:3001/api/auth/get-session",
            headers={"Cookie": f"better-auth.session_token={signed_token}"},
            timeout=5.0
        )
        print(f"Status: {response.status_code}")
        print(f"Response: {response.text[:500]}")

if __name__ == "__main__":
    asyncio.run(test_signed_token())
