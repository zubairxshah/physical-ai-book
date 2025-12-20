import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/Auth/SignupForm';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function SignupPage() {
  return (
    <Layout title="Sign Up" description="Create your Physical AI Book account">
      <BrowserOnly fallback={<div style={{minHeight: '60vh', display: 'flex', alignItems: 'center', justifyContent: 'center'}}>Loading...</div>}>
        {() => <SignupForm />}
      </BrowserOnly>
    </Layout>
  );
}
