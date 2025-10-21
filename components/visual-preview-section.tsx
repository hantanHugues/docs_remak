import { Sparkles } from "lucide-react"

export function VisualPreviewSection() {
  const images = [
    { title: "Bras robotique en action", query: "robotic arm manipulating objects with precision" },
    { title: "Interface de contrôle élégante", query: "modern web interface dashboard for robot control" },
    { title: "Système intégré fonctionnel", query: "integrated robotics system with multiple modules" },
    { title: "Équipe au travail", query: "engineering team working on robotics project" },
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

        <div className="grid md:grid-cols-2 gap-6">
          {images.map((image, index) => (
            <div
              key={index}
              className="group relative aspect-video rounded-2xl overflow-hidden border-2 border-primary/30 hover:border-primary/50 transition-all hover:scale-[1.02] shadow-xl hover:shadow-2xl"
            >
              <img
                src={`https://images.unsplash.com/600x400/?${encodeURIComponent(image.query)}`}
                alt={image.title}
                className="w-full h-full object-cover group-hover:scale-110 transition-transform duration-500"
              />
              <div className="absolute inset-0 bg-gradient-to-t from-black/80 via-black/20 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
              <div className="absolute bottom-0 left-0 right-0 p-6 translate-y-full group-hover:translate-y-0 transition-transform duration-300">
                <h3 className="text-xl font-bold text-white">{image.title}</h3>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  )
}
