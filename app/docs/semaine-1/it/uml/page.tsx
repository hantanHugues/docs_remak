"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Monitor, ArrowLeft, ArrowRight, AlertCircle, CheckCircle, BookOpen, Target } from "lucide-react"
import Link from "next/link"


export default function UMLPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Breadcrumb */}
          <AnimatedSection animation="fade-in">
            <div className="border-b border-border/60 bg-muted/30">
              <div className="container mx-auto px-4 py-4">
                <nav className="flex items-center space-x-2 text-sm text-muted-foreground">
                  <Link href="/docs/semaine-1/it" className="hover:text-foreground transition-colors">
                    <span>IT Semaine 1</span>
                  </Link>
                  <ArrowRight className="h-4 w-4" />
                  <span className="text-foreground font-medium">Diagrammes UML</span>
                </nav>
              </div>
            </div>
          </AnimatedSection>

          {/* Header */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-orange-50 via-white to-red-50 dark:from-orange-950/20 dark:via-background dark:to-red-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-xl bg-gradient-to-r from-orange-500 to-red-500 flex items-center justify-center">
                      <Monitor className="w-6 h-6 text-white" />
                    </div>
                    <div>
                      <Badge className="mb-2">Semaine 1 - Débutant</Badge>
                      <h1 className="text-3xl md:text-4xl font-bold">Diagrammes UML</h1>
                    </div>
                  </div>
                  <p className="text-xl text-muted-foreground leading-relaxed">
                    Maîtrisez la modélisation UML pour concevoir et documenter vos systèmes robotiques. 
                    Apprenez les diagrammes essentiels pour la robotique.
                  </p>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Content */}
          <div className="container mx-auto px-4 py-12">
            <div className="max-w-4xl mx-auto space-y-8">
              
              {/* Introduction */}
              <AnimatedSection animation="fade-up">
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Target className="w-5 h-5 text-orange-600" />
                      Objectifs d'apprentissage
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="grid md:grid-cols-2 gap-4">
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 mt-0.5 flex-shrink-0" />
                        <div>
                          <h4 className="font-semibold">Diagrammes de classes</h4>
                          <p className="text-sm text-muted-foreground">Structure des objets robotiques</p>
                        </div>
                      </div>
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 mt-0.5 flex-shrink-0" />
                        <div>
                          <h4 className="font-semibold">Diagrammes de séquence</h4>
                          <p className="text-sm text-muted-foreground">Interactions temporelles</p>
                        </div>
                      </div>
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 mt-0.5 flex-shrink-0" />
                        <div>
                          <h4 className="font-semibold">Diagrammes d'états</h4>
                          <p className="text-sm text-muted-foreground">Comportements dynamiques</p>
                        </div>
                      </div>
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 mt-0.5 flex-shrink-0" />
                        <div>
                          <h4 className="font-semibold">Documentation système</h4>
                          <p className="text-sm text-muted-foreground">Communication technique efficace</p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Qu'est-ce qu'UML */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle>Qu'est-ce qu'UML ?</CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <p>
                      <strong>UML (Unified Modeling Language)</strong> est un langage de modélisation standardisé 
                      pour visualiser, spécifier et documenter les systèmes logiciels complexes, incluant les systèmes robotiques.
                    </p>
                    <div className="bg-muted/50 p-4 rounded-lg">
                      <h4 className="font-semibold mb-2">Avantages pour la robotique</h4>
                      <ul className="space-y-1 text-sm">
                        <li>• <strong>Visualisation claire</strong> des architectures complexes</li>
                        <li>• <strong>Communication</strong> entre équipes multidisciplinaires</li>
                        <li>• <strong>Documentation</strong> standardisée et maintenable</li>
                        <li>• <strong>Planification</strong> avant l'implémentation</li>
                      </ul>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Diagramme de classes */}
              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Monitor className="w-5 h-5 text-orange-600" />
                      Diagramme de classes - Système robotique
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 text-slate-100 p-4 rounded-lg overflow-x-auto">
                      <pre className="text-sm">
{`┌─────────────────────────────────┐
│            Robot                │
├─────────────────────────────────┤
│ - nom: String                   │
│ - position: Position            │
│ - batterie: Float               │
│ - etat: EtatRobot               │
├─────────────────────────────────┤
│ + demarrer(): void              │
│ + arreter(): void               │
│ + deplacer(pos: Position): void │
│ + getBatterie(): Float          │
└─────────────────────────────────┘
                │
                │ hérite
                ▼
┌─────────────────────────────────┐
│         RobotMobile             │
├─────────────────────────────────┤
│ - vitesse: Float                │
│ - direction: Float              │
├─────────────────────────────────┤
│ + avancer(): void               │
│ + tourner(angle: Float): void   │
│ + naviguer(cible: Position)     │
└─────────────────────────────────┘
                │
                │ composition
                ▼
┌─────────────────────────────────┐
│           Capteur               │
├─────────────────────────────────┤
│ - type: TypeCapteur             │
│ - valeur: Float                 │
│ - precision: Float              │
├─────────────────────────────────┤
│ + lire(): Float                 │
│ + calibrer(): void              │
└─────────────────────────────────┘`}
                      </pre>
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Ce diagramme montre la hiérarchie des classes, leurs attributs, méthodes et relations.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Diagramme de séquence */}
              <AnimatedSection animation="fade-up" delay={300}>
                <Card>
                  <CardHeader>
                    <CardTitle>Diagramme de séquence - Navigation autonome</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 text-slate-100 p-4 rounded-lg overflow-x-auto">
                      <pre className="text-sm">
{`Utilisateur    Contrôleur    Robot    CapteurDistance    Moteurs
    │              │            │             │              │
    │─naviguer()──▶│            │             │              │
    │              │─demarrer()─▶│             │              │
    │              │            │─lire()─────▶│              │
    │              │            │◀─distance───│              │
    │              │◀─position──│             │              │
    │              │            │             │              │
    │              │─avancer()──▶│             │              │
    │              │            │─vitesse────────────────────▶│
    │              │            │             │              │
    │              │            │─lire()─────▶│              │
    │              │            │◀─obstacle───│              │
    │              │            │             │              │
    │              │─tourner()──▶│             │              │
    │              │            │─rotation───────────────────▶│
    │              │            │             │              │
    │              │─arreter()──▶│             │              │
    │              │            │─stop───────────────────────▶│
    │◀─terminé─────│            │             │              │`}
                      </pre>
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Ce diagramme illustre l'ordre chronologique des interactions entre objets.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Diagramme d'états */}
              <AnimatedSection animation="fade-up" delay={400}>
                <Card>
                  <CardHeader>
                    <CardTitle>Diagramme d'états - Comportement robot</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 text-slate-100 p-4 rounded-lg overflow-x-auto">
                      <pre className="text-sm">
{`                    ┌─────────────┐
                    │   Arrêté    │◀─── [État initial]
                    └─────────────┘
                           │
                    demarrer()
                           │
                           ▼
                    ┌─────────────┐
               ┌───▶│   Actif     │
               │    └─────────────┘
               │           │
               │    obstacle_detecte()
               │           │
               │           ▼
               │    ┌─────────────┐
               │    │ Évitement   │
               │    └─────────────┘
               │           │
               │    chemin_libre()
               │           │
               └───────────┘
                           │
                    batterie_faible()
                           │
                           ▼
                    ┌─────────────┐
                    │  Recharge   │
                    └─────────────┘
                           │
                    batterie_pleine()
                           │
                           ▼
                    ┌─────────────┐
                    │   Arrêté    │
                    └─────────────┘`}
                      </pre>
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Ce diagramme montre les différents états du robot et les transitions entre eux.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Cas d'usage robotique */}
              <AnimatedSection animation="fade-up" delay={500}>
                <Card>
                  <CardHeader>
                    <CardTitle>Cas d'usage - Système de surveillance</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 text-slate-100 p-4 rounded-lg overflow-x-auto">
                      <pre className="text-sm">
{`                    Système de Surveillance Robotique
                    
    Opérateur                                    Robot de Surveillance
        │                                              │
        │──── Programmer mission ─────────────────────▶│
        │                                              │
        │◀─── Confirmer mission ──────────────────────│
        │                                              │
        │──── Démarrer patrouille ────────────────────▶│
        │                                              │
        │◀─── Rapport de position ────────────────────│
        │                                              │
        │◀─── Alerte intrusion ───────────────────────│
        │                                              │
        │──── Investiguer zone ───────────────────────▶│
        │                                              │
        │◀─── Données capteurs ───────────────────────│
        │                                              │
        │──── Retour base ────────────────────────────▶│
        
    Capteurs inclus:
    • Caméra thermique
    • Détecteur de mouvement  
    • Capteur audio
    • GPS/Navigation`}
                      </pre>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Bonnes pratiques */}
              <AnimatedSection animation="fade-up" delay={600}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <AlertCircle className="w-5 h-5 text-orange-600" />
                      Bonnes pratiques UML en robotique
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="grid gap-4">
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-blue-100 dark:bg-blue-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <span className="text-xs font-bold text-blue-600">1</span>
                        </div>
                        <div>
                          <h4 className="font-semibold">Simplicité</h4>
                          <p className="text-sm text-muted-foreground">
                            Commencez par les diagrammes essentiels, ajoutez les détails progressivement
                          </p>
                        </div>
                      </div>
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-green-100 dark:bg-green-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <span className="text-xs font-bold text-green-600">2</span>
                        </div>
                        <div>
                          <h4 className="font-semibold">Cohérence</h4>
                          <p className="text-sm text-muted-foreground">
                            Utilisez les mêmes noms et conventions dans tous les diagrammes
                          </p>
                        </div>
                      </div>
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-purple-100 dark:bg-purple-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <span className="text-xs font-bold text-purple-600">3</span>
                        </div>
                        <div>
                          <h4 className="font-semibold">Mise à jour</h4>
                          <p className="text-sm text-muted-foreground">
                            Maintenez les diagrammes à jour avec l'évolution du code
                          </p>
                        </div>
                      </div>
                      <div className="flex items-start gap-3">
                        <div className="w-6 h-6 rounded-full bg-orange-100 dark:bg-orange-900/50 flex items-center justify-center flex-shrink-0 mt-0.5">
                          <span className="text-xs font-bold text-orange-600">4</span>
                        </div>
                        <div>
                          <h4 className="font-semibold">Collaboration</h4>
                          <p className="text-sm text-muted-foreground">
                            Partagez et validez les diagrammes avec l'équipe
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation */}
              <AnimatedSection animation="fade-up" delay={700}>
                <div className="flex justify-between items-center pt-8 border-t">
                  <Link href="/docs/semaine-1/it/wheeled-robot">
                    <Button variant="outline" className="gap-2">
                      <ArrowLeft className="w-4 h-4" />
                      Robot à Roues
                    </Button>
                  </Link>
                  <Link href="/docs/semaine-1/mecanique">
                    <Button className="gap-2">
                      Mécanique Semaine 1
                      <ArrowRight className="w-4 h-4" />
                    </Button>
                  </Link>
                </div>
              </AnimatedSection>
            </div>
          </div>
        </main>
      </div>
    </div>
  )
}
