/** @type {import('next').NextConfig} */
const nextConfig = {
  output: 'export',
  trailingSlash: true,
  // Configuration pour GitHub Pages - repo: 2025-Team-IFRI-Docs
  basePath: process.env.NODE_ENV === 'production' ? '/2025-Team-IFRI-Docs' : '',
  assetPrefix: process.env.NODE_ENV === 'production' ? '/2025-Team-IFRI-Docs/' : '',
  typescript: {
    ignoreBuildErrors: true,
  },
  images: {
    unoptimized: true,
  },
}

export default nextConfig
