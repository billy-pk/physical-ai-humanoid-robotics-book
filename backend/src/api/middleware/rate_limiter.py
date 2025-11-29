from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.requests import Request
from starlette.responses import Response, JSONResponse
from starlette.types import ASGIApp
import time
from collections import defaultdict

RATE_LIMIT_DURATION = 60  # seconds
RATE_LIMIT_REQUESTS = 10 # requests

class RateLimitMiddleware(BaseHTTPMiddleware):
    def __init__(self, app: ASGIApp):
        super().__init__(app)
        self.client_requests = defaultdict(list)

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        client_ip = request.client.host
        current_time = time.time()

        # Remove timestamps older than RATE_LIMIT_DURATION
        self.client_requests[client_ip] = [
            t for t in self.client_requests[client_ip]
            if t > current_time - RATE_LIMIT_DURATION
        ]

        if len(self.client_requests[client_ip]) >= RATE_LIMIT_REQUESTS:
            return JSONResponse(
                status_code=429,
                content={
                    "code": "TOO_MANY_REQUESTS",
                    "message": f"Rate limit exceeded: {RATE_LIMIT_REQUESTS} requests per {RATE_LIMIT_DURATION} seconds."
                },
            )

        self.client_requests[client_ip].append(current_time)
        response = await call_next(request)
        return response
