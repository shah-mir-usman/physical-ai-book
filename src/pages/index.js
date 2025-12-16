import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Home() {
  return (
    <Layout title="Home" description="Physical AI Textbook">
      <div className="hero-section">
        <h1 className="hero-title">Physical AI &<br/>Humanoid Robotics</h1>
        <p className="hero-subtitle">
          The Definitive Academic Curriculum for Embodied Intelligence,<br/>
          Control Theory, and Generative Robotics.
        </p>
        <Link to="/docs/intro" className="primary-btn">
          Start Reading
        </Link>
      </div>
    </Layout>
  );
}