"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Monitor, ArrowLeft, ArrowRight, AlertCircle, CheckCircle, BookOpen, Target } from "lucide-react"
import Link from "next/link"
import Image from "next/image"
import { MermaidDiagram } from "@/components/mermaid-diagram"


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

              {/* Diagramme de classes */}
              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle>Diagramme de classes - Hiérarchie robotique</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 p-4 rounded-lg">
                      <MermaidDiagram
                        chart={`
classDiagram
    class Robot {
        <<abstract>>
        -UUID id
        -str name
        -Tuple position
        -float orientation
        -str energy_source
        -int generator_level
        -bool is_active
        -List sensors
        +start() None
        +distance_to(other) float
        +consume_energy(amount) None
        +add_sensor(sensor) None
        +remove_sensor(sensor) None
        +move(direction, distance)* None
        +rotate(angle)* None
        +stop()* None
        +status()* str
    }
    
    class WheeledRobot {
        -float wheel_base
        -int storage_capacity
        -State state
        -List storage_bag
        -float obstacle_threshold
        -float v_lin_cmd
        -float v_ang_cmd
        +set_motor_speed(left, right) None
        +add_to_storage(item) bool
        +detect_obstacle(pos) bool
        +avoid_obstacle() bool
        +is_storage_full() bool
    }
    
    class RoboticArm {
        -List joint_angles
        -int num_joints
        -Tuple end_effector_position
        -Optional holding_item
        +set_joint_angle(index, angle) None
        +reset_arm() None
        +pick(item) None
        +place(item, pos) None
    }
    
    class State {
        <<enumeration>>
        IDLE
        NAVIGATING
        AVOIDING
        UPDATING_STORAGE
        RETURNING
        CHARGING
        SHUTDOWN
    }
    
    Robot <|-- WheeledRobot
    Robot <|-- RoboticArm
    WheeledRobot --> State
                        `}
                        className="w-full"
                      />
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Hiérarchie des classes basée sur la documentation : Robot (classe abstraite) avec ses 
                      implémentations WheeledRobot et RoboticArm, incluant leurs vrais attributs et méthodes.
                    </p>
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

              {/* Architecture système */}
              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <BookOpen className="w-5 h-5 text-blue-600" />
                      Architecture système robotique
                    </CardTitle>
                    <CardDescription>
                      Diagramme UML complet du système avec toutes les classes et leurs relations
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4 mb-4">
                      <Image 
                        src="/2025-Team-IFRI-Docs/Documentation/semaine-1/it/diagram.png" 
                        alt="Diagramme UML complet - Architecture robotique" 
                        width={800} 
                        height={600} 
                        className="w-full h-auto rounded-md"
                      />
                    </div>
                    <p className="text-sm text-muted-foreground">
                      Ce diagramme présente l'architecture complète du système robotique, illustrant la classe abstraite 
                      <strong> Robot</strong> et ses implémentations concrètes (<strong>WheeledRobot</strong>, <strong>RoboticArm</strong>), 
                      ainsi que leurs attributs, méthodes et relations d'héritage. Il montre également les interactions 
                      avec les capteurs et autres composants du système.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Diagramme de séquence */}
              <AnimatedSection animation="fade-up" delay={300}>
                <Card>
                  <CardHeader>
                    <CardTitle>Diagramme de séquence - WheeledRobot Navigation</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 p-4 rounded-lg">
                      <MermaidDiagram
                        chart={`
sequenceDiagram
    participant U as Utilisateur
    participant WR as WheeledRobot
    participant S as Sensors
    participant SB as StorageBag
    
    U->>WR: start()
    WR->>WR: is_active = True
    WR->>WR: state = IDLE
    
    U->>WR: move(direction, distance)
    WR->>WR: state = NAVIGATING
    WR->>WR: consume_energy(amount)
    WR->>WR: position = new_position
    
    U->>WR: detect_obstacle(position)
    WR->>S: check distance
    S-->>WR: distance < threshold
    WR->>WR: state = AVOIDING
    
    U->>WR: avoid_obstacle()
    WR->>WR: rotate(π/2)
    WR->>WR: move(forward, 0.5)
    WR->>WR: state = NAVIGATING
    
    U->>WR: add_to_storage(item)
    WR->>WR: state = UPDATING_STORAGE
    WR->>SB: append(item)
    SB-->>WR: storage_updated
    WR->>WR: state = IDLE
    
    U->>WR: stop()
    WR->>WR: is_active = False
    WR-->>U: status()
                        `}
                        className="w-full"
                      />
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Séquence d'interactions basée sur les vraies méthodes du WheeledRobot : start(), move(), 
                      detect_obstacle(), avoid_obstacle(), add_to_storage(), stop().
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Diagramme d'états */}
              <AnimatedSection animation="fade-up" delay={400}>
                <Card>
                  <CardHeader>
                    <CardTitle>Diagramme d'états - WheeledRobot</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 p-4 rounded-lg">
                      <MermaidDiagram
                        chart={`
stateDiagram-v2
    [*] --> IDLE : Robot initialized
    
    IDLE --> NAVIGATING : start_navigation()
    NAVIGATING --> AVOIDING : detect_obstacle()
    AVOIDING --> NAVIGATING : obstacle_cleared()
    NAVIGATING --> UPDATING_STORAGE : add_to_storage()
    UPDATING_STORAGE --> IDLE : storage_updated
    
    NAVIGATING --> RETURNING : storage_full()
    UPDATING_STORAGE --> RETURNING : storage_full()
    RETURNING --> CHARGING : battery_low()
    CHARGING --> IDLE : battery_full()
    
    IDLE --> SHUTDOWN : shutdown()
    NAVIGATING --> SHUTDOWN : shutdown()
    AVOIDING --> SHUTDOWN : shutdown()
    UPDATING_STORAGE --> SHUTDOWN : shutdown()
    RETURNING --> SHUTDOWN : shutdown()
    CHARGING --> SHUTDOWN : shutdown()
    
    SHUTDOWN --> [*]
    
    note right of IDLE
        Robot is ready
        and waiting for commands
    end note
    
    note right of NAVIGATING
        Robot is moving
        and consuming energy
    end note
    
    note right of AVOIDING
        Robot detected obstacle
        and performing avoidance
    end note
                        `}
                        className="w-full"
                      />
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Diagramme d'états du WheeledRobot basé sur l'enum State : IDLE, NAVIGATING, AVOIDING, 
                      UPDATING_STORAGE, RETURNING, CHARGING, SHUTDOWN.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Cas d'usage robotique */}
              <AnimatedSection animation="fade-up" delay={500}>
                <Card>
                  <CardHeader>
                    <CardTitle>Diagramme de cas d'usage - Système robotique</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-slate-900 p-4 rounded-lg">
                      <MermaidDiagram
                        chart={`
flowchart TD
    subgraph Système["🤖 Système Robotique"]
        subgraph Robot["Robot (ABC)"]
            UC1((Démarrer robot))
            UC2((Consommer énergie))
            UC3((Gérer capteurs))
            UC4((Calculer distance))
            UC5((Arrêter robot))
        end
        
        subgraph WR["WheeledRobot"]
            UC6((Naviguer))
            UC7((Éviter obstacles))
            UC8((Collecter objets))
            UC9((Gérer stockage))
            UC10((Contrôler moteurs))
            UC11((Retourner base))
        end
        
        subgraph RA["RoboticArm"]
            UC12((Contrôler joints))
            UC13((Saisir objets))
            UC14((Placer objets))
            UC15((Réinitialiser bras))
        end
    end
    
    User[👤 Utilisateur]
    Operator[👨‍💼 Opérateur]
    System[🖥️ Système de contrôle]
    
    User --- UC1
    User --- UC5
    User --- UC6
    User --- UC8
    User --- UC11
    
    Operator --- UC12
    Operator --- UC13
    Operator --- UC14
    Operator --- UC15
    
    System --- UC2
    System --- UC3
    System --- UC4
    System --- UC7
    System --- UC9
    System --- UC10
                        `}
                        className="w-full"
                      />
                    </div>
                    <p className="text-sm text-muted-foreground mt-4">
                      Diagramme de cas d'usage basé sur les vraies fonctionnalités documentées : Robot (classe abstraite), 
                      WheeledRobot (navigation, évitement, stockage) et RoboticArm (manipulation d'objets).
                    </p>
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
