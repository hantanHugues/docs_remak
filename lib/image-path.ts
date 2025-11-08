/**
 * Utilitaire pour gérer les chemins d'images avec basePath
 * Détecte automatiquement si on est sur GitHub Pages
 */

/**
 * Ajoute le basePath aux chemins d'images
 */
export function getImagePath(imagePath: string): string {
  // Détection côté client si on est sur GitHub Pages
  const isGitHubPages = typeof window !== 'undefined' && 
    (window.location.hostname === 'hantanhugues.github.io' || 
     window.location.pathname.startsWith('/2025-Team-IFRI-Docs'));
  
  const basePath = isGitHubPages ? '/2025-Team-IFRI-Docs' : '';
  
  // Si pas de basePath ou si l'image a déjà le basePath
  if (!basePath || imagePath.startsWith(basePath)) {
    return imagePath;
  }
  
  // Si l'image commence par /, on ajoute le basePath
  if (imagePath.startsWith('/')) {
    return `${basePath}${imagePath}`;
  }
  
  // Sinon on ajoute / et basePath
  return `${basePath}/${imagePath}`;
}
