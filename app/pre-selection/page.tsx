import Link from "next/link"
import { ArrowRight, Code, Cpu, Wrench, CheckCircle2, Calendar, Target, Clock, Users, Trophy, Zap } from "lucide-react"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import DotPattern from "@/components/ui/dot-pattern"
import { cn } from "@/lib/utils"
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper"
import { Navbar } from "@/components/navbar"
import { AnimatedSection } from "@/components/animated-section"

// Structure des données réelles basée sur la documentation
const weeksData = [
  {
    week: 1,
    title: "Tests Débutants",
    period: "05/06 au 11/06/2025",
    evaluation: "12/06/2025 (16h-18h)",
    description: "Tests de niveau débutant pour évaluer les compétences fondamentales",
    domains: {
      electronique: {
        title: "Gyroscope et accéléromètre",
        description: "Système de détection de mouvement avec communication I2C",
        tasks: [
          "Identifier et utiliser un capteur combinant gyroscope et accéléromètre",
          "Réaliser un circuit avec communication I2C",
          "Afficher les données sur un écran LCD",
          "Documenter le projet sur GitHub"
        ]
      },
      it: {
        title: "Gestion d'un Robot",
        description: "Programmation orientée objet et conception de classes",
        tasks: [
          "Création d'une classe Robot avec attributs et méthodes",
          "Implémentation de sous-classes spécialisées",
          "Utilisation de l'héritage et du polymorphisme",
          "Documentation avec schémas UML"
        ]
      },
      mecanique: {
        title: "Conception de base",
        description: "Compétences en CAO avec SolidWorks",
        tasks: [
          "Modélisation de pièces simples avec contraintes précises",
          "Calcul de masses avec différents matériaux",
          "Assemblage d'une pince mécanique",
          "Détermination des centres de gravité"
        ]
      }
    }
  },
  {
    week: 2,
    title: "Tests Intermédiaires",
    period: "12/06 au 18/06/2025",
    evaluation: "19/06/2025 (16h-18h)",
    description: "Tests de niveau intermédiaire pour des compétences plus avancées",
    domains: {
      electronique: {
        title: "La boîte noire",
        description: "Système d'enregistrement et transmission de données",
        tasks: [
          "Conception d'une boîte noire avec capteur de la semaine 1",
          "Transmission des données en temps réel via I2C",
          "Réalisation d'un PCB avec Atmega328P",
          "Station de contrôle avec affichage LCD"
        ]
      },
      it: {
        title: "Introduction à ROS2",
        description: "Compétences en robotique avec ROS2",
        tasks: [
          "Création d'un package ROS2 pour gestion de capteurs",
          "Implémentation de nodes publisher et subscriber",
          "Gestion des données de capteurs en temps réel",
          "Documentation et tests du système"
        ]
      },
      mecanique: {
        title: "Conception intermédiaire",
        description: "Compétences avancées en CAO",
        tasks: [
          "Modélisation de pièces complexes",
          "Gestion des modifications et contraintes",
          "Assemblage de maillons de chaîne",
          "Calculs de centres de masse"
        ]
      }
    }
  },
  {
    week: 3,
    title: "Tests Avancés",
    period: "19/06 au 25/06/2025",
    evaluation: "26/06/2025 (16h-18h)",
    description: "Tests de niveau avancé pour la maîtrise complète des compétences",
    domains: {
      electronique: {
        title: "Afficheur 7 segments avec servomoteurs",
        description: "Afficheur innovant avec servomoteurs",
        tasks: [
          "Conception d'un afficheur 7 segments utilisant des servomoteurs",
          "Réalisation d'un PCB avec Atmega328P",
          "Alimentation par batteries lithium",
          "Affichage des chiffres de 0 à 9"
        ]
      },
      it: {
        title: "Algorithme de Pathfinding",
        description: "Système de navigation autonome",
        tasks: [
          "Développement d'un algorithme de pathfinding (A*, Dijkstra, RRT)",
          "Intégration avec ROS2 et Gazebo",
          "Gestion de l'évitement d'obstacles",
          "Visualisation avec RViz2"
        ]
      },
      mecanique: {
        title: "Conception avancée",
        description: "Maîtrise complète de la CAO",
        tasks: [
          "Modélisation de pièces complexes avec paramètres variables",
          "Gestion des modifications multiples",
          "Calculs de masse précis",
          "Préparation pour l'impression 3D"
        ]
      }
    }
  }
]

const finalTest = {
  title: "Test Final : Système de Convoyeur",
  period: "26/06 au 09/07/2025",
  evaluation: "10/07/2025 (16h-19h)",
  description: "Système de convoyeur intelligent pour le tri de déchets - Projet multidisciplinaire",
  specs: {
    general: "Trier 4 types de déchets (cubes de couleurs : Vert, Jaune, Rouge, Bleu)",
    mecanique: "Convoyeur 650mm, hauteur 100mm, cubes 30mm",
    electronique: "ATmega328P/Arduino Nano, batteries lithium, capteur couleur",
    it: "Interface web temps réel, suivi quantités par type"
  },
  scoring: {
    electronique: "Circuiterie (40pts) + Code (15pts) + Fonctionnement (30pts) + Doc (10pts) + Présentation (5pts)",
    it: "Détection (20pts) + Automatisation (15pts) + Interface web (15pts) + Intégration (10pts) + Robustesse (15pts) + Doc (5pts) + Tests (5pts) + Créativité (5pts)",
    mecanique: "Conception (70pts) + Documentation (50pts) + Présentation (30pts)"
  }
}

// Icônes et couleurs par domaine
const domainConfig = {
  electronique: {
    icon: Cpu,
    color: "from-blue-500 to-cyan-500",
    bgColor: "from-blue-500/20 to-cyan-500/20",
    textColor: "text-blue-500"
  },
  it: {
    icon: Code,
    color: "from-purple-500 to-pink-500", 
    bgColor: "from-purple-500/20 to-pink-500/20",
    textColor: "text-purple-500"
  },
  mecanique: {
    icon: Wrench,
    color: "from-green-500 to-emerald-500",
    bgColor: "from-green-500/20 to-emerald-500/20", 
    textColor: "text-green-500"
  }
}

export default function PreSelectionPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Hero Section */}
          <section className="relative min-h-screen flex items-center justify-center overflow-hidden">
            {/* Arrière-plan teinté */}
            <div className="absolute inset-0 bg-gradient-to-br from-primary/15 via-accent/8 to-primary/10"></div>
            <DotPattern
              className={cn(
                "absolute inset-0 opacity-60",
                "[mask-image:radial-gradient(600px_circle_at_center,white,transparent)]",
              )}
            />
            <div className="container mx-auto px-4 relative z-10">
              <div className="max-w-4xl mx-auto text-center space-y-6">
                <Badge className="bg-primary/10 text-primary border-primary/30">
                  3 Semaines • 3 Domaines • 1 Finale
                </Badge>
                <h1 className="text-5xl md:text-7xl font-bold text-foreground">
                  Pré-sélection <span className="text-primary">TRC</span>
                </h1>
                <p className="text-xl text-muted-foreground max-w-2xl mx-auto leading-relaxed">
                  Progression technique à travers Électronique, IT et Mécanique vers l'excellence robotique
                </p>
                
                {/* Explication de la pré-sélection */}
                <div className="bg-card border border-border rounded-2xl p-6 max-w-3xl mx-auto">
                  <p className="text-base text-muted-foreground leading-relaxed mb-4">
                    <span className="font-semibold text-foreground">La pré-sélection</span> constitue la première étape du challenge TRC, 
                    permettant d'évaluer et de sélectionner les candidats les plus prometteurs. 
                    Seuls les participants ayant démontré leur excellence technique et leur capacité d'innovation 
                    accéderont à la <span className="font-semibold text-primary">finale prestigieuse</span>.
                  </p>
                  <div className="text-center">
                    <Link href="/finale">
                      <Button className="bg-gradient-to-r from-primary to-accent hover:from-primary/90 hover:to-accent/90 text-white">
                        Découvrir la finale
                        <ArrowRight className="ml-2 h-4 w-4" />
                      </Button>
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </section>

          {/* Overview Section */}
          <AnimatedSection animation="fade-in" delay={100}>
            <section className="py-16 bg-muted/30">
              <div className="container mx-auto px-4">
                <div className="max-w-6xl mx-auto grid md:grid-cols-3 gap-6">
                  <div className="group bg-card border border-border rounded-2xl p-8 hover:shadow-2xl hover:shadow-blue-500/10 transition-all duration-300 hover:-translate-y-2">
                    <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-blue-500/20 to-cyan-500/20 flex items-center justify-center mb-4 group-hover:scale-110 transition-transform">
                      <Calendar className="h-7 w-7 text-blue-500" />
                    </div>
                    <h3 className="text-xl font-bold mb-2">3 Semaines</h3>
                    <p className="text-muted-foreground leading-relaxed">
                      Progression du niveau débutant à avancé
                    </p>
                  </div>
                  <div className="group bg-card border border-border rounded-2xl p-8 hover:shadow-2xl hover:shadow-purple-500/10 transition-all duration-300 hover:-translate-y-2">
                    <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-purple-500/20 to-pink-500/20 flex items-center justify-center mb-4 group-hover:scale-110 transition-transform">
                      <Target className="h-7 w-7 text-purple-500" />
                    </div>
                    <h3 className="text-xl font-bold mb-2">3 Domaines</h3>
                    <p className="text-muted-foreground leading-relaxed">
                      Électronique, IT et Mécanique
                    </p>
                  </div>
                  <div className="group bg-card border border-border rounded-2xl p-8 hover:shadow-2xl hover:shadow-green-500/10 transition-all duration-300 hover:-translate-y-2">
                    <div className="w-14 h-14 rounded-xl bg-gradient-to-br from-green-500/20 to-emerald-500/20 flex items-center justify-center mb-4 group-hover:scale-110 transition-transform">
                      <Trophy className="h-7 w-7 text-green-500" />
                    </div>
                    <h3 className="text-xl font-bold mb-2">Test Final</h3>
                    <p className="text-muted-foreground leading-relaxed mb-4">
                      Projet multidisciplinaire intégré
                    </p>
                    <Link href="/finale">
                      <Button variant="outline" size="sm" className="w-full group-hover:bg-green-500/10 group-hover:border-green-500/30 transition-all">
                        Voir la finale
                        <ArrowRight className="ml-2 h-3 w-3" />
                      </Button>
                    </Link>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Semaines par domaines */}
          <section className="py-20">
            <div className="container mx-auto px-4">
              <div className="max-w-7xl mx-auto">
                <div className="text-center mb-16">
                  <h2 className="text-4xl md:text-5xl font-bold mb-4">
                    Progression par <span className="text-primary">Semaines</span>
                  </h2>
                  <p className="text-xl text-muted-foreground max-w-3xl mx-auto">
                    Chaque semaine développe vos compétences dans les trois domaines essentiels
                  </p>
                </div>

                <div className="space-y-20">
                  {weeksData.map((week, idx) => (
                    <AnimatedSection key={week.week} animation="fade-in" delay={idx * 200}>
                      <div id={`semaine-${week.week}`} className="relative">
                        {/* Timeline indicator */}
                        <div className="absolute -left-4 top-0 bottom-0 w-1 bg-gradient-to-b from-primary/50 to-accent/50 rounded-full" />
                        
                        <div className="pl-8">
                          {/* Week Header */}
                          <div className="mb-8">
                            <div className="flex items-center gap-4 mb-4">
                              <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-primary to-accent flex items-center justify-center text-white font-bold text-lg">
                                {week.week}
                              </div>
                              <div>
                                <h3 className="text-3xl font-bold">{week.title}</h3>
                                <p className="text-muted-foreground">{week.description}</p>
                              </div>
                            </div>
                            <div className="flex flex-wrap gap-4 text-sm">
                              <Badge variant="outline" className="gap-2">
                                <Calendar className="w-4 h-4" />
                                {week.period}
                              </Badge>
                              <Badge variant="outline" className="gap-2">
                                <Clock className="w-4 h-4" />
                                Évaluation : {week.evaluation}
                              </Badge>
                            </div>
                          </div>

                          {/* Domaines */}
                          <div className="grid lg:grid-cols-3 gap-8">
                            {Object.entries(week.domains).map(([domainKey, domain]) => {
                              const config = domainConfig[domainKey as keyof typeof domainConfig]
                              const Icon = config.icon
                              
                              return (
                                <div
                                  key={domainKey}
                                  className="bg-card border border-border rounded-xl p-6 hover:shadow-lg hover:shadow-primary/10 transition-all duration-300 hover:-translate-y-1"
                                >
                                  <div className="flex items-center gap-3 mb-4">
                                    <div className={`w-10 h-10 rounded-lg bg-gradient-to-br ${config.bgColor} flex items-center justify-center`}>
                                      <Icon className={`h-5 w-5 ${config.textColor}`} />
                                    </div>
                                    <div>
                                      <h4 className="font-bold text-lg capitalize">{domainKey}</h4>
                                      <p className="text-sm text-muted-foreground">{domain.title}</p>
                                    </div>
                                  </div>
                                  
                                  <p className="text-muted-foreground mb-4 text-sm leading-relaxed">
                                    {domain.description}
                                  </p>
                                  
                                  <div className="space-y-2">
                                    <h5 className="font-semibold text-sm">Objectifs :</h5>
                                    <ul className="space-y-1">
                                      {domain.tasks.map((task, taskIdx) => (
                                        <li key={taskIdx} className="flex items-start gap-2 text-xs text-muted-foreground">
                                          <CheckCircle2 className="w-3 h-3 text-green-500 mt-0.5 flex-shrink-0" />
                                          <span>{task}</span>
                                        </li>
                                      ))}
                                    </ul>
                                  </div>
                                </div>
                              )
                            })}
                          </div>
                        </div>
                      </div>
                    </AnimatedSection>
                  ))}
                </div>
              </div>
            </div>
          </section>

          {/* Test Final */}
          <AnimatedSection animation="fade-in" delay={600}>
            <section id="test-final" className="py-20 bg-gradient-to-br from-primary/5 via-accent/5 to-primary/5">
              <div className="container mx-auto px-4">
                <div className="max-w-6xl mx-auto">
                  <div className="text-center mb-12">
                    <Badge className="bg-gradient-to-r from-primary to-accent text-white border-0 shadow-lg shadow-primary/30 mb-6">
                      Test Final Multidisciplinaire
                    </Badge>
                    <h2 className="text-4xl md:text-5xl font-bold mb-6">
                      <span className="bg-gradient-to-r from-primary to-accent bg-clip-text text-transparent">
                        {finalTest.title}
                      </span>
                    </h2>
                    <p className="text-xl text-muted-foreground mb-8 leading-relaxed">
                      {finalTest.description}
                    </p>
                    <div className="flex flex-wrap justify-center gap-4 text-sm">
                      <Badge variant="outline" className="gap-2">
                        <Calendar className="w-4 h-4" />
                        {finalTest.period}
                      </Badge>
                      <Badge variant="outline" className="gap-2">
                        <Clock className="w-4 h-4" />
                        Présentation : {finalTest.evaluation}
                      </Badge>
                    </div>
                  </div>

                  <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6 mb-12">
                    <div className="bg-card border border-border rounded-xl p-6">
                      <h4 className="font-bold mb-2">Objectif</h4>
                      <p className="text-sm text-muted-foreground">{finalTest.specs.general}</p>
                    </div>
                    <div className="bg-card border border-border rounded-xl p-6">
                      <h4 className="font-bold mb-2 text-green-500">Mécanique</h4>
                      <p className="text-sm text-muted-foreground">{finalTest.specs.mecanique}</p>
                    </div>
                    <div className="bg-card border border-border rounded-xl p-6">
                      <h4 className="font-bold mb-2 text-blue-500">Électronique</h4>
                      <p className="text-sm text-muted-foreground">{finalTest.specs.electronique}</p>
                    </div>
                    <div className="bg-card border border-border rounded-xl p-6">
                      <h4 className="font-bold mb-2 text-purple-500">IT</h4>
                      <p className="text-sm text-muted-foreground">{finalTest.specs.it}</p>
                    </div>
                  </div>

                  <div className="text-center">
                    <Link href="/finale">
                      <Button size="lg" className="bg-gradient-to-r from-primary to-accent hover:from-primary/90 hover:to-accent/90 text-white shadow-lg shadow-primary/30">
                        Découvrir le test final
                        <ArrowRight className="ml-2 h-5 w-5" />
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
