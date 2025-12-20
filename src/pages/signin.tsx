import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/Auth/SigninForm';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function SigninPage() {
  return (
    <BrowserOnly>
      {() => (
        <Layout title="Sign In" description="Sign in to your Physical AI Book account">
          <SigninForm />
        </Layout>
      )}
    </BrowserOnly>
  );
}
