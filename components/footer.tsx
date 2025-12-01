import { Github, Mail, ExternalLink } from "lucide-react"
import Image from "next/image"

export function Footer() {
  return (
    <footer className="border-t border-border bg-card">
      <div className="container mx-auto px-4 py-12">
        {/* Section Partenaires */}
        <div className="mb-12 pb-8 border-b border-border/50">
          <h4 className="text-center text-sm font-semibold text-muted-foreground mb-6 uppercase tracking-wide">
            Nos Partenaires
          </h4>
          <div className="flex items-center justify-center gap-12 flex-wrap">
            <div className="relative group">
              <div className="absolute inset-0 bg-gradient-to-r from-primary/20 to-chart-1/20 rounded-lg blur-xl opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
              <div className="relative bg-background/50 backdrop-blur-sm p-4 rounded-lg border border-border/50 hover:border-primary/30 transition-all duration-300">
                <Image
                  src="/2025-Team-IFRI-Docs/IFRI-300x300.png"
                  alt="IFRI - École des Participants"
                  width={80}
                  height={80}
                  className="object-contain mix-blend-multiply dark:mix-blend-screen dark:opacity-90"
                />
                <p className="text-xs text-center mt-2 text-muted-foreground">École IFRI</p>
              </div>
            </div>
            <div className="relative group">
              <div className="absolute inset-0 bg-gradient-to-r from-primary/20 to-chart-1/20 rounded-lg blur-xl opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
              <div className="relative bg-background/50 backdrop-blur-sm p-4 rounded-lg border border-border/50 hover:border-primary/30 transition-all duration-300">
                <Image
                  src="/2025-Team-IFRI-Docs/téléchargé.png"
                  alt="TRC - Organisateurs"
                  width={100}
                  height={60}
                  className="object-contain"
                />
                <p className="text-xs text-center mt-2 text-muted-foreground">Organisateurs TRC</p>
              </div>
            </div>
          </div>
        </div>

        <div className="grid md:grid-cols-3 gap-8 mb-8">
          <div className="space-y-4">
            <h3 className="text-xl font-bold">IFRI TRC Docs</h3>
            <p className="text-sm text-muted-foreground leading-relaxed">
              Documentation complète de notre parcours dans la compétition robotique TRC 2024.
            </p>
          </div>

          <div className="space-y-4">
            <h4 className="font-semibold">Navigation</h4>
            <ul className="space-y-2 text-sm text-muted-foreground">
              <li>
                <a href="#" className="hover:text-primary transition-colors flex items-center gap-2">
                  Pré-sélection
                  <ExternalLink className="w-3 h-3" />
                </a>
              </li>
              <li>
                <a href="#" className="hover:text-primary transition-colors flex items-center gap-2">
                  Projet Final
                  <ExternalLink className="w-3 h-3" />
                </a>
              </li>
              <li>
                <a href="#" className="hover:text-primary transition-colors flex items-center gap-2">
                  Équipe
                  <ExternalLink className="w-3 h-3" />
                </a>
              </li>
              <li>
                <a href="#" className="hover:text-primary transition-colors flex items-center gap-2">
                  Galerie
                  <ExternalLink className="w-3 h-3" />
                </a>
              </li>
            </ul>
          </div>

          <div className="space-y-4">
            <h4 className="font-semibold">Contact</h4>
            <div className="flex gap-4">
              <a
                href="#"
                className="w-10 h-10 rounded-lg bg-primary/10 hover:bg-primary/20 flex items-center justify-center text-primary transition-colors"
                aria-label="GitHub"
              >
                <Github className="w-5 h-5" />
              </a>
              <a
                href="#"
                className="w-10 h-10 rounded-lg bg-primary/10 hover:bg-primary/20 flex items-center justify-center text-primary transition-colors"
                aria-label="Email"
              >
                <Mail className="w-5 h-5" />
              </a>
            </div>
          </div>
        </div>

        <div className="pt-8 border-t border-border text-center text-sm text-muted-foreground">
          <p>© 2025 TRC Competition Documentation. Créé avec passion pour l'innovation robotique.</p>
        </div>
      </div>
    </footer>
  )
}
