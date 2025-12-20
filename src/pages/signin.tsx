import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/Auth/SigninForm';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function SigninPage() {
  return (
    <Layout title="Sign In" description="Sign in to your Physical AI Book account">
      <BrowserOnly fallback={<div style={{minHeight: '60vh', display: 'flex', alignItems: 'center', justifyContent: 'center'}}>Loading...</div>}>
        {() => <SigninForm />}
      </BrowserOnly>
    </Layout>
  );
}
