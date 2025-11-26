"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { PageNavigation } from "@/components/page-navigation";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, Workflow, PlayCircle
} from "lucide-react";
import Link from "next/link";

export default function SensorLaunchDocPage() {
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
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Documentation Informatique
                    </Link>
                    <span>/</span>
                    <span>sensor_launch.py</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Code className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold"><code>sensor_launch.py</code></h1>
                      <p className="text-muted-foreground">Fichier de lancement ROS 2 pour démarrer les nœuds de simulation de capteur.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">ROS 2</Badge>
                    <Badge variant="outline">Launch File</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <PageNavigation />

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <AnimatedSection animation="fade-up">
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Description générale
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Ce script définit un fichier <strong>launch</strong> ROS 2 en Python qui permet de <strong>lancer simultanément</strong> deux nœuds :
                      </p>
                      <ul className="list-disc pl-5 my-4 space-y-1 text-sm">
                        <li><code>sensor_publisher</code> : génère des données simulées,</li>
                        <li><code>sensor_subscriber</code> : consomme et analyse ces données.</li>
                      </ul>
                      <p>
                        C’est une façon pratique d’exécuter les deux programmes avec une seule commande.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Workflow className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Détail par bloc de code
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Analyse de chaque partie du script de lancement.
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    
                    {/* Bloc 1 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        1- Importation des modules
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`from launch import LaunchDescription
from launch_ros.actions import Node`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400">
                          <li><code>LaunchDescription</code> : décrit le contenu du lancement,</li>
                          <li><code>Node</code> : permet de définir un nœud ROS à lancer.</li>
                        </ul>
                      </div>
                    </div>
                    
                    {/* Bloc 2 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        2- Fonction <code>generate_launch_description</code>
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def generate_launch_description():`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <p className="text-gray-600 dark:text-gray-400">
                          Point d’entrée pour le système de lancement ROS 2. Elle retourne la description des actions à effectuer au démarrage.
                        </p>
                      </div>
                    </div>

                    {/* Bloc 3 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        3- Déclaration des nœuds
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`    return LaunchDescription([
        Node(
            package='sensor_data_evaluation',
            executable='sensor_publisher',
            name='sensor_publisher'
        ),
        Node(
            package='sensor_data_evaluation',
            executable='sensor_subscriber',
            name='sensor_subscriber'
        ),
    ])`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <ul className="list-disc pl-5 space-y-2 text-gray-600 dark:text-gray-400">
                          <li>Crée une liste de deux actions :
                            <ul className="list-['-_'] pl-5 mt-1">
                                <li>Lancement du nœud <code>sensor_publisher</code>,</li>
                                <li>Lancement du nœud <code>sensor_subscriber</code>,</li>
                            </ul>
                          </li>
                          <li>Chaque <code>Node</code> précise :
                            <ul className="list-['-_'] pl-5 mt-1 space-y-1">
                                <li><code>package</code> : le nom du package ROS (<code>sensor_data_evaluation</code>),</li>
                                <li><code>executable</code> : le script à lancer (<code>sensor_publisher</code> ou <code>sensor_subscriber</code>),</li>
                                <li><code>name</code> : nom attribué au nœud ROS.</li>
                            </ul>
                          </li>
                        </ul>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Commande d’exécution
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                      <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                      <pre className="text-sm text-gray-100"><code>{`source install/setup.bash
ros2 launch sensor_data_evaluation sensor_launch.py`}</code></pre>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-2/it/ros2-sensor-evaluation">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    ROS2 Sensor Evaluation
                  </Button>
                </Link>
                <Link href="/docs/semaine-2/it/sensor-publisher">
                  <Button>
                    Sensor Publisher
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