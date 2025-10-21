import Link from "next/link"
import { ArrowLeft, Bot, Eye, Network, Package, Monitor, Zap, Layers, Workflow, Target } from "lucide-react"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import DotPattern from "@/components/ui/dot-pattern"
import Particles from "@/components/ui/particles"
import { cn } from "@/lib/utils"
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper"
import { Navbar } from "@/components/navbar"
import { AnimatedSection } from "@/components/animated-section"

const modules = [
  {
    name: "DofBot",
    icon: Bot,
    color: "from-blue-500 to-cyan-500",
    tagline: "Le bras qui manipule avec précision",
    description:
      "Bras robotique à 5 degrés de liberté capable de manipuler des objets avec une précision millimétrique",
    features: [
      "Cinématique inverse pour positionnement précis",
      "Contrôle en temps réel des servomoteurs",
      "Préhension adaptative d'objets variés",
      "Interface de programmation intuitive",
    ],
  },
  {
    name: "Jetson Nano",
    icon: Eye,
    color: "from-green-500 to-emerald-500",
    tagline: "L'œil intelligent qui voit et comprend",
    description: "Système de vision par ordinateur alimenté par l'IA pour la détection et reconnaissance d'objets",
    features: [
      "Détection d'objets en temps réel",
      "Reconnaissance de formes et couleurs",
      "Traitement d'image GPU accéléré",
      "Modèles d'apprentissage profond optimisés",
    ],
  },
  {
    name: "ROS Master",
    icon: Network,
    color: "from-purple-500 to-pink-500",
    tagline: "Le chef d'orchestre qui coordonne",
    description: "Système central de coordination utilisant ROS pour orchestrer tous les modules du système",
    features: [
      "Communication inter-modules en temps réel",
      "Gestion des états et transitions",
      "Synchronisation des actions",
      "Architecture distribuée robuste",
    ],
  },
  {
    name: "Convoyeur",
    icon: Package,
    color: "from-orange-500 to-red-500",
    tagline: "Le transporteur automatisé",
    description: "Système de convoyage intelligent pour le transport automatisé d'objets entre les stations",
    features: [
      "Contrôle de vitesse variable",
      "Détection de présence d'objets",
      "Synchronisation avec le bras robotique",
      "Gestion de flux optimisée",
    ],
  },
  {
    name: "Interface Web",
    icon: Monitor,
    color: "from-pink-500 to-rose-500",
    tagline: "Le tableau de bord de contrôle",
    description: "Interface web moderne pour le monitoring et le contrôle en temps réel de tout le système",
    features: [
      "Visualisation en temps réel des données",
      "Contrôle manuel et automatique",
      "Logs et historique des opérations",
      "Dashboard responsive et intuitif",
    ],
  },
]


export default function FinalePage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Hero Section */}
          <section className="relative py-20 overflow-hidden">
            <Particles className="absolute inset-0 opacity-40" quantity={100} />
            <DotPattern
              className={cn(
                "absolute inset-0 opacity-20",
                "[mask-image:radial-gradient(700px_circle_at_center,white,transparent)]",
              )}
            />
            <div className="container mx-auto px-4 relative z-10">
              <div className="max-w-4xl mx-auto text-center space-y-6">
                <Badge className="bg-gradient-to-r from-primary/20 to-accent/20 text-primary border-primary/30 shadow-lg shadow-primary/20">
                  5 Modules Interconnectés
                </Badge>
                <h1 className="text-5xl md:text-7xl font-bold">
                  <span className="bg-gradient-to-r from-primary via-accent to-primary bg-clip-text text-transparent animate-gradient">
                    Finale TRC
                  </span>
                </h1>
                <p className="text-xl text-muted-foreground max-w-2xl mx-auto leading-relaxed">
                  L'aboutissement du projet : un système robotique complet et intégré démontrant l'excellence technique
                </p>
              </div>
            </div>
          </section>

          {/* System Overview */}
          <AnimatedSection animation="fade-in" delay={100}>
            <section className="py-16 bg-muted/30">
              <div className="container mx-auto px-4">
                <div className="max-w-6xl mx-auto">
                  <div className="text-center mb-12">
                    <h2 className="text-3xl md:text-4xl font-bold mb-4">Architecture du Système</h2>
                    <p className="text-muted-foreground max-w-2xl mx-auto leading-relaxed">
                      Un écosystème robotique complet où chaque module communique et collabore pour accomplir des tâches
                      complexes
                    </p>
                  </div>

                  <div className="grid md:grid-cols-3 gap-6">
                    <div className="group bg-card border border-border rounded-2xl p-8 hover:shadow-2xl hover:shadow-blue-500/10 transition-all duration-300 hover:-translate-y-2">
                      <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-blue-500/20 to-cyan-500/20 flex items-center justify-center mb-4 group-hover:scale-110 transition-transform">
                        <Layers className="h-7 w-7 text-blue-500" />
                      </div>
                      <h3 className="text-xl font-bold mb-2">Intégration Complète</h3>
                      <p className="text-muted-foreground leading-relaxed">
                        Tous les modules travaillent ensemble de manière synchronisée
                      </p>
                    </div>
                    <div className="group bg-card border border-border rounded-2xl p-8 hover:shadow-2xl hover:shadow-purple-500/10 transition-all duration-300 hover:-translate-y-2">
                      <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-purple-500/20 to-pink-500/20 flex items-center justify-center mb-4 group-hover:scale-110 transition-transform">
                        <Workflow className="h-7 w-7 text-purple-500" />
                      </div>
                      <h3 className="text-xl font-bold mb-2">Automatisation</h3>
                      <p className="text-muted-foreground leading-relaxed">
                        Flux de travail automatisé de bout en bout
                      </p>
                    </div>
                    <div className="group bg-card border border-border rounded-2xl p-8 hover:shadow-2xl hover:shadow-green-500/10 transition-all duration-300 hover:-translate-y-2">
                      <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-green-500/20 to-emerald-500/20 flex items-center justify-center mb-4 group-hover:scale-110 transition-transform">
                        <Zap className="h-7 w-7 text-green-500" />
                      </div>
                      <h3 className="text-xl font-bold mb-2">Temps Réel</h3>
                      <p className="text-muted-foreground leading-relaxed">Communication et traitement en temps réel</p>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Modules Section */}
          <section className="py-20">
            <div className="container mx-auto px-4">
              <div className="max-w-6xl mx-auto space-y-16">
                {modules.map((module, idx) => {
                  const Icon = module.icon
                  const isEven = idx % 2 === 0
                  let moduleId = module.name.toLowerCase().replace(/\s+/g, "-")
                  if (moduleId === "jetson-nano") moduleId = "jetson"
                  if (moduleId === "ros-master") moduleId = "ros"
                  if (moduleId === "interface-web") moduleId = "interface"

                  return (
                    <AnimatedSection key={idx} animation={isEven ? "slide-right" : "slide-left"} delay={100}>
                      <div
                        id={moduleId}
                        className={cn("grid md:grid-cols-2 gap-12 items-center", !isEven && "md:grid-flow-dense")}
                      >
                        <div className={cn("space-y-6", !isEven && "md:col-start-2")}>
                          <div className="flex items-center gap-4">
                            <div
                              className={cn(
                                "w-16 h-16 rounded-2xl bg-gradient-to-br flex items-center justify-center shadow-xl",
                                module.color,
                              )}
                            >
                              <Icon className="h-8 w-8 text-white" />
                            </div>
                            <div>
                              <h2 className="text-3xl md:text-4xl font-bold">{module.name}</h2>
                              <p className="text-muted-foreground italic text-sm">{module.tagline}</p>
                            </div>
                          </div>

                          <p className="text-lg leading-relaxed text-muted-foreground">{module.description}</p>

                          <div className="space-y-3">
                            {module.features.map((feature, fIdx) => (
                              <div key={fIdx} className="flex items-start gap-3 group">
                                <div
                                  className={cn(
                                    "w-6 h-6 rounded-lg bg-gradient-to-br flex items-center justify-center flex-shrink-0 mt-0.5 group-hover:scale-110 transition-transform",
                                    module.color,
                                  )}
                                >
                                  <div className="w-2 h-2 bg-white rounded-full" />
                                </div>
                                <p className="text-muted-foreground leading-relaxed">{feature}</p>
                              </div>
                            ))}
                          </div>
                        </div>

                        <div className={cn("relative", !isEven && "md:col-start-1 md:row-start-1")}>
                          <div className="relative aspect-video rounded-2xl overflow-hidden border border-border bg-muted/50 backdrop-blur-sm shadow-xl hover:shadow-2xl transition-shadow">
                            <div className={cn("absolute inset-0 bg-gradient-to-br opacity-10", module.color)} />
                            <div className="absolute inset-0 flex items-center justify-center">
                              <Icon className="h-32 w-32 text-muted-foreground/20" />
                            </div>
                          </div>
                        </div>
                      </div>
                    </AnimatedSection>
                  )
                })}
              </div>
            </div>
          </section>

          {/* CTA Section */}
          <AnimatedSection animation="scale" delay={100}>
            <section className="py-20 bg-gradient-to-br from-muted/50 to-muted/30">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto text-center space-y-6">
                  <h2 className="text-4xl md:text-5xl font-bold">De l'apprentissage à l'excellence</h2>
                  <p className="text-xl text-muted-foreground leading-relaxed">
                    Ce système représente l'aboutissement de semaines de travail acharné et d'innovation technique
                  </p>
                  <div className="flex flex-wrap gap-4 justify-center">
                    <Link href="/pre-selection">
                      <Button
                        variant="outline"
                        size="lg"
                        className="gap-2 bg-transparent hover:scale-105 transition-all"
                      >
                        <ArrowLeft className="h-4 w-4" />
                        Voir la pré-sélection
                      </Button>
                    </Link>
                    <Link href="/">
                      <Button
                        size="lg"
                        className="gap-2 bg-gradient-to-r from-primary to-accent hover:opacity-90 shadow-xl shadow-primary/30 hover:scale-105 transition-all"
                      >
                        Retour à l'accueil
                      </Button>
                    </Link>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>
        </main>
      </div>
    </div>
  )
}
