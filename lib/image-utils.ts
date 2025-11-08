/**
 * Utilitaire pour gérer les chemins d'images avec basePath
 */

// Récupère le basePath depuis la configuration Next.js
const basePath = process.env.NODE_ENV === 'production' ? '/2025-Team-IFRI-Docs' : '';

/**
 * Ajoute le basePath aux chemins d'images
 * @param imagePath - Chemin relatif de l'image (ex: "/Documentation/...")
 * @returns Chemin complet avec basePath
 */
export function getImagePath(imagePath: string): string {
  // Si l'image commence déjà par le basePath, on ne l'ajoute pas
  if (imagePath.startsWith(basePath)) {
    return imagePath;
  }
  
  // Ajoute le basePath au début
  return `${basePath}${imagePath}`;
}

/**
 * Composant Image wrapper qui gère automatiquement le basePath
 */
import Image, { ImageProps } from 'next/image';

interface SmartImageProps extends Omit<ImageProps, 'src'> {
  src: string;
}

export function SmartImage({ src, ...props }: SmartImageProps) {
  return <Image src={getImagePath(src)} {...props} />;
}
