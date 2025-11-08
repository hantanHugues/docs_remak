"use client"

import { useEffect } from 'react'

export function ImageFix() {
  useEffect(() => {
    // Seulement en production (GitHub Pages)
    if (process.env.NODE_ENV === 'production') {
      // Trouve toutes les images qui commencent par /Documentation/
      const images = document.querySelectorAll('img[src^="/Documentation/"]')
      
      images.forEach((img) => {
        const htmlImg = img as HTMLImageElement
        if (!htmlImg.src.includes('/2025-Team-IFRI-Docs/')) {
          // Ajoute le préfixe /2025-Team-IFRI-Docs au src
          htmlImg.src = htmlImg.src.replace('/Documentation/', '/2025-Team-IFRI-Docs/Documentation/')
        }
      })
    }
  }, [])

  return null // Ce composant ne rend rien visuellement
}
