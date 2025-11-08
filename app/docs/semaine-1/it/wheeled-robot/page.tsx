"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Bot, Code, List, Settings, Shapes, Puzzle, BookOpen, Target, PlayCircle, HardHat, Car, ShieldAlert, Package
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';


export default function WheeledRobotDocFRPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-white dark:bg-gray-900 border-b border-gray-200 dark:border-gray-700">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="/docs/semaine-1/it" className="hover:text-foreground transition-colors">
                      Documentation Informatique
                    </Link>
                    <span>/</span>
                    <span>Classe : WheeledRobot</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Car className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Classe : <code>WheeledRobot</code></h1>
                      <p className="text-muted-foreground">Documentation technique du robot mobile à locomotion terrestre.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">Informatique</Badge>
                    <Badge variant="outline">POO</Badge>
                    <Badge variant="outline">Robotique Mobile</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="#">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Page Précédente
                  </Button>
                </Link>
                <Link href="#">
                  <Button variant="ghost" size="sm">
                    Page Suivante
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <AnimatedSection animation="fade-up">
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Bot className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Présentation de l'architecture
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Implémentation concrète spécialisée pour la robotique mobile terrestre
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-4">
                      <div className="not-prose mb-6 flex justify-center">
                        <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/it/wheeled-robot.svg" alt="Robot à Roues" width={100} height={100} className="opacity-80" />
                      </div>
                      <p className="text-gray-600 dark:text-gray-400 leading-relaxed">
                        La classe <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">WheeledRobot</code> est une implémentation concrète de la classe de base abstraite <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">Robot</code>. Elle représente un robot mobile qui se déplace à l'aide de roues et est capable de navigation, d'évitement d'obstacles de base et de transporter des objets dans un compartiment de stockage.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Objectifs pédagogiques et fonctionnels
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Finalités techniques et comportements attendus du système
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-3">
                      <div className="flex items-start gap-3">
                        <div className="w-2 h-2 rounded-full bg-gray-400 dark:bg-gray-500 mt-2 flex-shrink-0"></div>
                        <span className="text-gray-600 dark:text-gray-400">Simuler un robot terrestre avec une locomotion à roues</span>
                      </div>
                      <div className="flex items-start gap-3">
                        <div className="w-2 h-2 rounded-full bg-gray-400 dark:bg-gray-500 mt-2 flex-shrink-0"></div>
                        <span className="text-gray-600 dark:text-gray-400">Fournir des fonctionnalités pour le mouvement (avant, arrière), la rotation et le contrôle de la vitesse des moteurs</span>
                      </div>
                      <div className="flex items-start gap-3">
                        <div className="w-2 h-2 rounded-full bg-gray-400 dark:bg-gray-500 mt-2 flex-shrink-0"></div>
                        <span className="text-gray-600 dark:text-gray-400">Gérer un système de stockage pour les objets collectés</span>
                      </div>
                      <div className="flex items-start gap-3">
                        <div className="w-2 h-2 rounded-full bg-gray-400 dark:bg-gray-500 mt-2 flex-shrink-0"></div>
                        <span className="text-gray-600 dark:text-gray-400">Implémenter la détection d'obstacles de base et les manœuvres d'évitement</span>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                        <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                            <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                            Énumération : <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">State</code>
                        </CardTitle>
                        <CardDescription className="text-gray-600 dark:text-gray-400">
                          États opérationnels et modes de fonctionnement du robot mobile
                        </CardDescription>
                    </CardHeader>
                    <CardContent className="p-6">
                        <p className="text-gray-600 dark:text-gray-400 mb-4">
                          Définit les états opérationnels possibles du <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">WheeledRobot</code>.
                        </p>
                        <div className="space-y-3">
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">IDLE</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot n'effectue aucune tâche spécifique</span>
                              </div>
                            </div>
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">NAVIGATING</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot se déplace activement vers une cible ou le long d'un chemin</span>
                              </div>
                            </div>
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">AVOIDING</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot effectue une manœuvre d'évitement d'obstacle</span>
                              </div>
                            </div>
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">UPDATING_STORAGE</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot est en train d'ajouter un objet à son stockage</span>
                              </div>
                            </div>
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">RETURNING</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot navigue pour retourner à une base ou une station de charge (conceptuel)</span>
                              </div>
                            </div>
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">CHARGING</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot charge actuellement son générateur (conceptuel)</span>
                              </div>
                            </div>
                            <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                              <div className="flex items-start gap-3">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono flex-shrink-0">SHUTDOWN</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">Le robot est éteint ou dans un état non opérationnel</span>
                              </div>
                            </div>
                        </div>
                    </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Settings className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Propriétés et attributs spécialisés
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Extensions et spécialisations de l'interface Robot
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4 mb-6">
                      <p className="text-sm text-gray-600 dark:text-gray-400 mb-0">
                        <strong>Héritage :</strong> Hérite de toutes les propriétés de <code>Robot</code> (<code>id</code>, <code>name</code>, <code>position</code>, <code>orientation</code>, <code>energy_source</code>, <code>generator_level</code>, <code>is_active</code>, <code>sensors</code>).
                      </p>
                    </div>
                    
                    <div className="space-y-6">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">wheel_base</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Distance entre les centres des roues du robot, paramètre fondamental pour la cinématique.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">float</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Unité :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Mètres</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule</span>
                          </div>
                        </div>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">storage_capacity</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Nombre maximum d'objets que le robot peut stocker dans son compartiment de charge.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">int</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule</span>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">state</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          État opérationnel actuel du robot selon l'énumération State.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">State</code> (Enum)
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture-écriture</span>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">storage_bag</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Collection représentant les objets actuellement transportés par le robot.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">List[Any]</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule (modifié via add_to_storage)</span>
                          </div>
                        </div>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">obstacle_threshold</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Distance minimale à laquelle un objet est considéré comme un obstacle nécessitant évitement.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">float</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Unité :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Mètres</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture-écriture</span>
                          </div>
                        </div>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">v_lin_cmd</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Commande de vitesse linéaire courante du robot mobile.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">float</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Unité :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">m/s</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule (défini via set_motor_speed)</span>
                          </div>
                        </div>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                          <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">v_ang_cmd</code>
                        </h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                          Commande de vitesse angulaire courante du robot mobile.
                        </p>
                        <div className="grid md:grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Type :</span>
                            <code className="ml-2 bg-gray-100 dark:bg-gray-800 px-1 rounded">float</code>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Unité :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">rad/s</span>
                          </div>
                          <div>
                            <span className="font-medium text-gray-700 dark:text-gray-300">Accès :</span>
                            <span className="ml-2 text-gray-600 dark:text-gray-400">Lecture seule (défini via set_motor_speed)</span>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Code className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Constructeur et initialisation
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">__init__(name, position, orientation, energy_source, wheel_base, storage_capacity)</code>
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="space-y-6">
                      <div>
                        <p className="text-gray-600 dark:text-gray-400 mb-4">
                          Initialise une nouvelle instance de <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">WheeledRobot</code>.
                        </p>
                        
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`def __init__(
    self,
    name: str,
    position: Tuple[float, float],
    orientation: float,
    energy_source: str,
    wheel_base: float,
    storage_capacity: int,
) -> None:
    """
    Initialise le robot mobile avec les paramètres donnés.
    ...
    """
    super().__init__(name=name, position=position, orientation=orientation, energy_source=energy_source)

    if not isinstance(wheel_base, (int, float)) or wheel_base <= 0:
        raise ValueError("wheel_base doit être un nombre positif.")
    if not isinstance(storage_capacity, int) or storage_capacity <= 0:
        raise ValueError("storage_capacity doit être un entier positif.")
    
    self._wheel_base = float(wheel_base)
    self._v_lin_cmd = 0.0 # Commande de vitesse linéaire (m/s)
    self._v_ang_cmd = 0.0 # Commande de vitesse angulaire (rad/s)

    self._stockage_capacity = storage_capacity
    self._storage_bag = []

    self._state = State.IDLE
    self._obstacle_threshold = 0.3 # Distance min pour considérer un obstacle (mètres)
`}</code></pre>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Paramètres d'entrée
                        </h4>
                        <div className="space-y-3">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">name</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Identifiant textuel unique du robot mobile</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">position</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Position (x, y) initiale en mètres dans le référentiel global</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">orientation</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Orientation initiale en radians par rapport à l'axe X</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">energy_source</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Source d'énergie validée contre l'énumération Robot.ENERGY_SOURCE</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">wheel_base</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Distance entre les roues, doit être un nombre positif</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">storage_capacity</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Capacité de stockage maximale, doit être un entier positif</span>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Exceptions levées
                        </h4>
                        <div className="space-y-2">
                          <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-red-100 dark:bg-red-900 px-2 py-1 rounded text-sm font-mono">ValueError</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Si wheel_base ou storage_capacity ne sont pas positifs, ou si energy_source est invalide</span>
                              </div>
                            </div>
                          </div>
                          <div className="bg-red-50 dark:bg-red-950/20 border border-red-200 dark:border-red-800 rounded p-3">
                            <div className="flex items-start gap-3">
                              <code className="bg-red-100 dark:bg-red-900 px-2 py-1 rounded text-sm font-mono">TypeError</code>
                              <div className="flex-1">
                                <span className="text-sm text-gray-600 dark:text-gray-400">Pour des types de paramètres invalides (validation parent ou directe)</span>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div>
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                          Exemple d'utilisation
                        </h4>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`from robot import Robot
from wheeledRobot import WheeledRobot, State
import math

rover = WheeledRobot(
    name="Mars Rover II",
    orientation=0.0,
    energy_source="electric", 
    wheel_base=0.5,
    storage_capacity=10
)
print(f"Robot créé : {rover.name} avec base de roues {rover.wheel_base}m")
print(f"Capacité de stockage : {rover.storage_capacity} objets")
print(f"État initial : {rover.state}")`}</code></pre>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Résumé des propriétés et accesseurs
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Interface publique des propriétés avec comportements associés
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                      <table className="w-full">
                        <thead className="bg-gray-50 dark:bg-gray-800">
                          <tr>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Propriété</th>
                            <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Comportement fonctionnel</th>
                          </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">wheel_base</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Largeur de l'empattement en lecture seule (immutable après construction)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">storage_capacity</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Capacité maximale de stockage en lecture seule (immutable après construction)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">state</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Getter/setter avec validation par rapport à l'énumération State</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">storage_bag</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Retourne une copie des objets actuellement stockés (lecture seule)</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">obstacle_threshold</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Getter/setter pour le seuil de proximité d'évitement d'obstacles</td>
                          </tr>
                          <tr>
                            <td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">v_lin_cmd / v_ang_cmd</td>
                            <td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Commandes de vitesse linéaire et angulaire en lecture seule (contrôlées par set_motor_speed)</td>
                          </tr>
                        </tbody>
                      </table>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Shapes className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Interface des méthodes d'instance
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Implémentations des méthodes abstraites et fonctionnalités spécialisées
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    {/* Method: set_motor_speed */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">set_motor_speed(left, right)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Définit les vitesses désirées pour les roues gauche et droite, qui déterminent à leur tour la vélocité linéaire et angulaire du robot.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def set_motor_speed(self, left: float, right: float) -> None:
    """
    Define motors speeds for left and right wheels.

    Args:
        left (float): Speed for the left wheel (m/s).
        right (float): Speed for the right wheel (m/s).
    Raises:
        ValueError: If left or right speed is not a number.
    """
    if not isinstance(left, (int, float)):
        raise ValueError("left speed must be a number.")
    if not isinstance(right, (int, float)):
        raise ValueError("right speed must be a number.")

    self._v_lin_cmd = (left + right) / 2.0
    self._v_ang_cmd = (right - left) / self.wheel_base
    self._logger.info(f"Motors set: v={self._v_lin_cmd:.2f}, ω={self._v_ang_cmd:.2f}")`}</code></pre>
                      </div>

                      <div>
                        <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Exemple d'utilisation</h5>
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                          <pre className="text-sm text-gray-100"><code>{`rover.start()
rover.set_motor_speed(left=0.2, right=0.2)
# Vitesse linéaire : 0.20 m/s, Vitesse angulaire : 0.00 rad/s
rover.set_motor_speed(left=0.1, right=-0.1)
# Vitesse linéaire : 0.00 m/s, Vitesse angulaire : -0.40 rad/s`}</code></pre>
                        </div>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: add_to_storage */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">add_to_storage(item)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Ajoute un objet au compartiment de stockage du robot s'il y a de la place disponible.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def add_to_storage(self, item: Any) -> bool:
    """
    Add an item to the robot's storage bag.
    
    Args:
        item (Any): The item to be added.
    
    Returns:
        bool: True if the item was added, False if storage is full.
    """
    if len(self._storage_bag) >= self._stockage_capacity:
        self._logger.warning("Storage is full, cannot add item.")
        return False
    self.state = State.UPDATING_STORAGE
    self._storage_bag.append(item)
    self._logger.info(f"Item added to storage. Current count: {len(self._storage_bag)}")
    return True`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: move */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">move(direction, distance)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Déplace le robot vers l'avant ou l'arrière sur une distance spécifiée (implémentation de la méthode abstraite).
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def move(self, direction: str, distance: float) -> None:
    if not self.is_active:
        self._logger.warning("Cannot move: Robot is inactive.")
        return
    if not isinstance(distance, (int, float)):
        raise ValueError("distance must be a number.")
    if distance < 0:
        raise ValueError("distance must be a non-negative number.")

    if direction not in ["forward", "backward"]:
        self._logger.error("Invalid move direction. Use 'forward', 'backward'")
        raise ValueError("move direction must be 'forward', 'backward'.")

    self.state = State.NAVIGATING
    angle = self.orientation if direction == "forward" else self.orientation + math.pi

    # Compute new position based on distance and angle
    dx = distance * math.cos(angle)
    dy = distance * math.sin(angle)
    new_position = (self.position[0] + dx, self.position[1] + dy)

    self._consume_for_motion(distance)
    self.position = new_position

    self._logger.info(f"Moved {direction} by {distance:.2f}m to {self.position}")`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: rotate */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">rotate(angle)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Fait pivoter le robot vers une nouvelle orientation absolue (implémentation de la méthode abstraite).
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def rotate(self, angle: float) -> None:
    """
    Fait pivoter le robot vers une orientation absolue.
    """
    self.orientation = angle`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: stop */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">stop()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Arrête tout mouvement du robot et le désactive (implémentation de la méthode abstraite).
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def stop(self) -> None:
    """
    Arrête tous les mouvements du robot.
    """
    self._v_lin_cmd = 0.0
    self._v_ang_cmd = 0.0
    self.is_active = False`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: detect_obstacle */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">detect_obstacle(obstacle_position)</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Vérifie si un obstacle est détecté dans la distance limite définie par obstacle_threshold.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def detect_obstacle(self, obstacle_position: Tuple[float, float]) -> bool:
    """
    Check if an obstacle is detected within the threshold distance.
    """
    if not isinstance(obstacle_position, tuple) or len(obstacle_position) != 2:
        raise ValueError("obstacle_position must be a tuple (x, y).")
    
    distance = self.distance_to(obstacle_position)
    if distance < self._obstacle_threshold:
        self._logger.info(f"Obstacle detected at {obstacle_position} (distance: {distance:.2f}m)")
        return True
    return False`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: avoid_obstacle */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">avoid_obstacle()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Manœuvre simple d'évitement d'obstacle par rotation et déplacement latéral.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def avoid_obstacle(self) -> bool:
    """
    Manœuvre d'évitement simple.
    """
    self.rotate(math.pi / 2)  # Pivoter de 90 degrés
    self.move("forward", 0.5)  # Avancer de 0.5 mètres
    return True`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: is_storage_full */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">is_storage_full()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Vérifie si le compartiment de stockage du robot a atteint sa capacité maximale.
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def is_storage_full(self) -> bool:
    """
    Vérifie si le stockage est plein.
    """
    return len(self._storage_bag) >= self._stockage_capacity`}</code></pre>
                      </div>
                    </div>
                    
                    <Separator className="my-6" />
                    
                    {/* Method: status */}
                    <div className="space-y-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                        <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">status()</code>
                      </h4>
                      <p className="text-gray-600 dark:text-gray-400">
                        Fournit une chaîne de caractères résumant le statut actuel du robot mobile (implémentation de la méthode abstraite).
                      </p>
                      
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def status(self) -> str:
    """
    Get the current status of the robot.
    Returns:
        str: A string describing the robot's status.
    """
    status = (
        f"Robot '{self.name}' Status:\\n"
        f"Position: {self.position}\\n"
        f"Orientation: {self.orientation:.2f} radians\\n"
        f"Energy Level: {self.generator_level:.2f}\\n"
        f"State: {self.state.value}\\n"
        f"Storage Capacity: {len(self._storage_bag)}/{self._stockage_capacity}\\n"
        f"Linear Speed Command: {self._v_lin_cmd:.2f} m/s\\n"
        f"Angular Speed Command: {self._v_ang_cmd:.2f} rad/s\\n"
    )
    return status`}</code></pre>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                        <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                            <Puzzle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                            Méthodes protégées (usage interne)
                        </CardTitle>
                        <CardDescription className="text-gray-600 dark:text-gray-400">
                          Fonctionnalités internes pour la gestion énergétique
                        </CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-4 p-6">
                        <div className="space-y-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">_consume_for_motion(distance)</code>
                            </h4>
                            <p className="text-gray-600 dark:text-gray-400">
                              Calcule et consomme l'énergie proportionnellement à la distance parcourue.
                            </p>
                        </div>
                        
                        <Separator className="my-4" />
                        
                        <div className="space-y-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-sm font-mono">_consume_for_rotation(angle)</code>
                            </h4>
                            <p className="text-gray-600 dark:text-gray-400">
                              Calcule et consomme l'énergie proportionnellement à l'angle de rotation effectué.
                            </p>
                        </div>
                    </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Scénario d'Exemple : Navigation et Collecte
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Démonstration complète des fonctionnalités du robot mobile terrestre
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-600 dark:text-gray-400 mb-4">
                      Ce scénario démontre la création d'un <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">WheeledRobot</code>, son déplacement, la détection d'un objet (simulée) et son ajout au stockage.
                    </p>
                    <div className="bg-blue-50 dark:bg-blue-950/20 border border-blue-200 dark:border-blue-800 rounded p-4">
                      <p className="text-sm text-gray-900 dark:text-gray-100">
                        <strong>Ressource additionnelle :</strong> Consultez les tests unitaires pour des exemples d'utilisation détaillés dans <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">test_wheeledrobot.py</code>
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation footer */}
              <div className="flex items-center justify-between pt-8 border-t border-gray-200 dark:border-gray-700">
                <Link href="../robotic-arm">
                  <Button variant="outline" className="text-gray-600 dark:text-gray-400 border-gray-300 dark:border-gray-600 hover:bg-gray-50 dark:hover:bg-gray-800">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Bras Robotique
                  </Button>
                </Link>
                <Link href="#">
                  <Button className="bg-gray-900 dark:bg-gray-100 text-white dark:text-gray-900 hover:bg-gray-800 dark:hover:bg-gray-200">
                    Module Suivant
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>
        </main>
      </div>
    </div>
  )
}