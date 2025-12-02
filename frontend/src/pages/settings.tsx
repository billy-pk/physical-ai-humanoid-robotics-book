import React from 'react';
import Layout from '@theme/Layout';
import PreferenceManagement from '../components/Auth/PreferenceManagement'; // Assuming this path

const SettingsPage: React.FC = () => {
  return (
    <Layout title="Settings" description="Manage your user settings and preferences.">
      <main>
        <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', padding: '2rem' }}>
          <div style={{ maxWidth: '800px', width: '100%' }}>
            <PreferenceManagement />
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default SettingsPage;
