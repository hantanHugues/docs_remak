import { Card, CardContent } from "@/components/ui/card"
import { Users, Award, Target, Lightbulb } from "lucide-react"

export function ParticipantsSection() {
  const highlights = [
    {
      icon: Users,
      title: "Équipe Collaborative",
      description: "Une synergie parfaite entre électronique, mécanique et informatique",
      color: "text-chart-1",
    },
    {
      icon: Target,
      title: "Objectifs Ambitieux",
      description: "Repousser les limites de l'innovation robotique",
      color: "text-chart-4",
    },
    {
      icon: Lightbulb,
      title: "Solutions Créatives",
      description: "Des approches innovantes pour résoudre des défis complexes",
      color: "text-chart-5",
    },
    {
      icon: Award,
      title: "Excellence Technique",
      description: "Un engagement total vers la qualité et la performance",
      color: "text-primary",
    },
  ]

  return (
    <section className="py-24 px-4 bg-gradient-to-b from-background to-secondary/10">
      <div className="container mx-auto max-w-6xl">
        <div className="text-center space-y-4 mb-16">
          <h2 className="text-4xl md:text-5xl font-bold text-balance">
            L'Esprit de <span className="text-primary">Compétition</span>
          </h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto text-balance leading-relaxed">
            Notre participation à la TRC Competition reflète notre passion pour la robotique et notre volonté de
            repousser les limites
          </p>
        </div>

        <div className="grid md:grid-cols-2 gap-6 mb-12">
          {highlights.map((highlight, index) => (
            <Card
              key={index}
              className="group hover:shadow-xl transition-all duration-300 hover:-translate-y-1 border-2 hover:border-primary/50"
            >
              <CardContent className="p-6 flex items-start gap-4">
                <div
                  className={`w-12 h-12 rounded-xl bg-primary/10 flex items-center justify-center ${highlight.color} flex-shrink-0 group-hover:scale-110 transition-transform`}
                >
                  <highlight.icon className="w-6 h-6" />
                </div>
                <div className="space-y-2">
                  <h3 className="text-xl font-bold">{highlight.title}</h3>
                  <p className="text-muted-foreground leading-relaxed">{highlight.description}</p>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>

        <div className="relative overflow-hidden rounded-2xl bg-gradient-to-br from-primary via-chart-2 to-chart-1 p-12 text-center text-primary-foreground">
          <div className="relative z-10 space-y-6">
            <h3 className="text-3xl md:text-4xl font-bold">Prêt à Découvrir Notre Projet ?</h3>
            <p className="text-lg opacity-90 max-w-2xl mx-auto">
              Explorez notre documentation complète et découvrez comment nous avons transformé des défis techniques en
              solutions innovantes
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center pt-4">
              <button className="px-8 py-3 bg-background text-foreground rounded-lg font-semibold hover:scale-105 transition-transform shadow-lg">
                Documentation Complète
              </button>
              <button className="px-8 py-3 bg-background/10 backdrop-blur-sm border-2 border-background/30 rounded-lg font-semibold hover:bg-background/20 transition-colors">
                Galerie Photos
              </button>
            </div>
          </div>

          {/* Decorative elements */}
          <div className="absolute top-0 right-0 w-64 h-64 bg-background/10 rounded-full blur-3xl" />
          <div className="absolute bottom-0 left-0 w-64 h-64 bg-background/10 rounded-full blur-3xl" />
        </div>
      </div>
    </section>
  )
}
