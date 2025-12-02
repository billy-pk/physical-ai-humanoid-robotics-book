import React from 'react';
import Markdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

interface FullViewProps {
  content: string;
}

const FullView: React.FC<FullViewProps> = ({ content }) => {
  return (
    <div className="full-content-view">
      <Markdown remarkPlugins={[remarkGfm]}>{content}</Markdown>
    </div>
  );
};

export default FullView;
