import { NextResponse } from 'next/server'
import type { NextRequest } from 'next/server'

export function middleware(request: NextRequest) {
  const url = request.nextUrl.clone()
  
  // Redirection des images Documentation vers le bon chemin avec basePath
  if (url.pathname.startsWith('/Documentation/')) {
    url.pathname = `/2025-Team-IFRI-Docs${url.pathname}`
    return NextResponse.rewrite(url)
  }
  
  // Redirection des images placeholder vers le bon chemin avec basePath
  if (url.pathname.match(/\.(jpg|jpeg|png|gif|svg|webp)$/)) {
    if (!url.pathname.startsWith('/2025-Team-IFRI-Docs/') && !url.pathname.startsWith('/_next/')) {
      url.pathname = `/2025-Team-IFRI-Docs${url.pathname}`
      return NextResponse.rewrite(url)
    }
  }
  
  return NextResponse.next()
}

export const config = {
  matcher: [
    // Matcher pour Documentation
    '/Documentation/:path*',
    // Matcher pour les images placeholder
    '/placeholder-:path*.jpg',
    '/placeholder-:path*.png',
    '/placeholder-:path*.svg',
  ],
}
