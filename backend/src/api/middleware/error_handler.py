from fastapi import Request, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint

from ...core.logging import logger
from ...models.errors import ErrorResponse

class ErrorHandlingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self,
                       request: Request,
                       call_next: RequestResponseEndpoint) -> JSONResponse:
        try:
            response = await call_next(request)
            return response
        except Exception as exc:
            logger.error("Unhandled exception", exc=str(exc), path=request.url.path)
            error_response = ErrorResponse(
                code="INTERNAL_SERVER_ERROR",
                message="An unexpected error occurred."
            )
            return JSONResponse(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                content=error_response.model_dump()
            )