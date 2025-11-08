import { Sparkles } from "lucide-react"
import Image from "next/image"
import { getImagePath } from "@/lib/image-path"

export function VisualPreviewSection() {
  const images = [
    { 
      title: "Système Convoyeur Automatisé", 
      src: "/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v3.png",
      description: "Convoyeur robotique avec système de tri automatique"
    },
    { 
      title: "Dashboard Streamlit Interactif", 
      src: "/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/dash_normal.png",
      description: "Interface de monitoring en temps réel"
    },
    { 
      title: "PCB 3D - Cube de Vol", 
      src: "/2025-Team-IFRI-Docs/Documentation/semaine-2/electronique/images/cube_pcb_PCB_3DViewer1.png",
      description: "Circuit imprimé pour système embarqué"
    },
    { 
      title: "Assemblage Mécanique", 
      src: "/2025-Team-IFRI-Docs/Documentation/semaine-2/mecanique/assets/imgs/picture_assemblage_a.png",
      description: "Assemblage robotique avec centre de masse calculé"
    },
    { 
      title: "Schéma Électronique KiCad", 
      src: "/2025-Team-IFRI-Docs/Documentation/semaine-2/electronique/images/station_schema.png",
      description: "Conception de circuit pour station de base"
    },
    { 
      title: "Processus de Fabrication CAO", 
      src: "/2025-Team-IFRI-Docs/Documentation/semaine-3/mecanique/assets/imgs/img_15.png",
      description: "Étapes de modélisation et validation"
    },
  ]

  return (
    <section className="relative py-24 px-4 overflow-hidden">
      <div className="absolute inset-0 bg-gradient-to-b from-secondary/20 via-background to-background" />

      <div className="container relative mx-auto max-w-6xl">
        <div className="text-center space-y-4 mb-16">
          <div className="inline-flex items-center justify-center gap-2 mb-4">
            <Sparkles className="w-6 h-6 text-primary" />
            <span className="text-sm font-semibold text-primary uppercase tracking-wider">Aperçu Visuel</span>
          </div>
          <h2 className="text-4xl md:text-5xl lg:text-6xl font-bold text-balance">
            Un Aperçu de{" "}
            <span className="bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
              Nos Réalisations
            </span>
          </h2>
          <p className="text-lg text-muted-foreground italic">Chaque projet raconte une histoire d'innovation</p>
        </div>

        <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
          {images.map((image, index) => (
            <div
              key={index}
              className="group relative aspect-video rounded-2xl overflow-hidden border-2 border-primary/30 hover:border-primary/50 transition-all hover:scale-[1.02] shadow-xl hover:shadow-2xl"
            >
              <Image
                src={image.src}
                alt={image.title}
                width={600}
                height={400}
                className="w-full h-full object-cover group-hover:scale-110 transition-transform duration-500"
                unoptimized
              />
              <div className="absolute inset-0 bg-gradient-to-t from-black/80 via-black/20 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
              <div className="absolute bottom-0 left-0 right-0 p-6 translate-y-full group-hover:translate-y-0 transition-transform duration-300">
                <h3 className="text-xl font-bold text-white">{image.title}</h3>
                <p className="text-sm text-gray-200 mt-2">{image.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  )
}
