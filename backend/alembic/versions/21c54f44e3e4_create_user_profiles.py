"""create_user_profiles

Revision ID: 21c54f44e3e4
Revises: b20f7b5c7f26
Create Date: 2025-11-30 01:26:44.523705

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = '21c54f44e3e4'
down_revision: Union[str, Sequence[str], None] = 'b20f7b5c7f26'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Create user_profiles table for storing user background and profile information."""
    op.create_table(
        'user_profiles',
        sa.Column('user_id', sa.Text(), nullable=False),  # Text to match Better Auth user.id
        sa.Column('software_background', sa.JSON(), nullable=True),  # Array of software skills
        sa.Column('hardware_background', sa.JSON(), nullable=True),  # Array of hardware skills
        sa.Column('experience_level', sa.String(20), nullable=True),  # beginner/intermediate/advanced
        sa.Column('learning_goals', sa.Text(), nullable=True),  # Free text, max 500 chars
        sa.Column('has_robotics_projects', sa.Boolean(), nullable=True),
        sa.Column('robotics_projects_description', sa.Text(), nullable=True),
        sa.Column('programming_years', sa.Integer(), nullable=True),  # 0-50
        sa.Column('learning_style', sa.String(20), nullable=True),  # visual/hands-on/theoretical/mixed
        sa.Column('questionnaire_completed', sa.Boolean(), server_default='false', nullable=False),
        sa.Column('questionnaire_completed_at', sa.TIMESTAMP(timezone=True), nullable=True),
        sa.Column('preferences', sa.JSON(), nullable=True),  # For future extensibility
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.PrimaryKeyConstraint('user_id'),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE')
    )
    
    # Create indexes for common queries
    op.create_index('ix_user_profiles_user_id', 'user_profiles', ['user_id'])
    op.create_index('ix_user_profiles_questionnaire_completed', 'user_profiles', ['questionnaire_completed'])


def downgrade() -> None:
    """Drop user_profiles table."""
    op.drop_index('ix_user_profiles_questionnaire_completed', table_name='user_profiles')
    op.drop_index('ix_user_profiles_user_id', table_name='user_profiles')
    op.drop_table('user_profiles')
