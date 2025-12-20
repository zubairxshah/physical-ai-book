import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/Auth/SignupForm';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function SignupPage() {
  return (
    <BrowserOnly>
      {() => (
        <Layout title="Sign Up" description="Create your Physical AI Book account">
          <SignupForm />
        </Layout>
      )}
    </BrowserOnly>
  );
}
