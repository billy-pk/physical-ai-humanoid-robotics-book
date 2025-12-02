"""create_personalized_content_cache_table

Revision ID: 7f2815e9e91e
Revises: 954501217eb6
Create Date: 2025-12-01 20:15:01.261651

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql


# revision identifiers, used by Alembic.
revision: str = '7f2815e9e91e'
down_revision: Union[str, Sequence[str], None] = '954501217eb6'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    op.create_table(
        'personalized_content_cache',
        sa.Column('cache_id', postgresql.UUID(as_uuid=True), primary_key=True,
                  server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', sa.TEXT(), nullable=False),
        sa.Column('chapter_id', sa.VARCHAR(255), nullable=False),
        sa.Column('content_type', sa.VARCHAR(20), nullable=False),
        sa.Column('user_preferences_hash', sa.VARCHAR(64), nullable=False),
        sa.Column('chapter_content_hash', sa.VARCHAR(64), nullable=False),
        sa.Column('target_language', sa.VARCHAR(10), nullable=True),
        sa.Column('generated_content', sa.TEXT(), nullable=False),
        sa.Column('generation_metadata', postgresql.JSONB(), nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.func.now(), nullable=False),
        sa.Column('last_accessed_at', sa.TIMESTAMP(), server_default=sa.func.now(), nullable=False),
        sa.Column('access_count', sa.INTEGER(), server_default='1', nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE')
    )

    # Unique constraint for cache key
    op.create_unique_constraint(
        'uq_personalized_cache',
        'personalized_content_cache',
        ['user_id', 'chapter_id', 'content_type', 'user_preferences_hash',
         'chapter_content_hash', 'target_language']
    )

    # Indexes
    op.create_index(
        'idx_personalized_cache_lookup',
        'personalized_content_cache',
        ['user_id', 'chapter_id', 'content_type', 'user_preferences_hash']
    )
    op.create_index(
        'idx_personalized_cache_cleanup',
        'personalized_content_cache',
        ['last_accessed_at']
    )


def downgrade() -> None:
    op.drop_table('personalized_content_cache')
