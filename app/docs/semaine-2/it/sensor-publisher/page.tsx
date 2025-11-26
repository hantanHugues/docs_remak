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
  ArrowLeft, ArrowRight, Code, BookOpen, Workflow, PlayCircle, Upload
} from "lucide-react";
import Link from "next/link";

export default function SensorPublisherDocPage() {
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
                    <span>sensor_publisher.py</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Upload className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold"><code>sensor_publisher.py</code></h1>
                      <p className="text-muted-foreground">Nœud ROS 2 pour la publication périodique de données de capteur simulées.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">ROS 2</Badge>
                    <Badge variant="outline">Publisher</Badge>
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
                        Ce script définit un <strong>nœud ROS 2</strong> nommé <code>sensor_publisher</code> qui <strong>publie périodiquement</strong> des mesures <strong>simulées</strong> de température, humidité et pression toutes les <strong>0.5 secondes</strong> sur le topic <code>/sensor_data</code>.
                      </p>
                      <p className="mt-4">
                        Les données sont envoyées sous la forme d’un message <code>Float32MultiArray</code> contenant trois valeurs flottantes correspondant à <strong>[température, humidité, pression]</strong>.
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
                      Analyse de chaque partie du script du publisher.
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    
                    {/* Bloc 1 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        1- Importations
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import random`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction : Importe les modules nécessaires</p>
                        <ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400">
                          <li><code>rclpy</code> : pour manipuler les nœuds ROS 2,</li>
                          <li><code>Float32MultiArray</code> : message ROS pour transmettre un vecteur de float,</li>
                          <li><code>random</code> : génération aléatoire de données simulées.</li>
                        </ul>
                      </div>
                    </div>
                    
                    {/* Bloc 2 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        2- Définition de la classe <code>SensorPublisher</code>
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`class SensorPublisher(Node):`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <p className="text-gray-600 dark:text-gray-400">
                          Crée un nœud ROS 2 dérivé de <code>Node</code>, qui publiera des données simulées.
                        </p>
                      </div>
                    </div>

                    {/* Bloc 3 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        3- Constructeur <code>__init__</code>
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def __init__(self):
    super().__init__('sensor_publisher')
    self.publisher_ = self.create_publisher(Float32MultiArray, '/sensor_data', 10)
    timer_period = 0.5
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('SensorPublisher initialisé, publication toutes les 0.5s.')`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400">
                          <li>Initialise le nœud sous le nom <code>sensor_publisher</code>,</li>
                          <li>Crée un publisher ROS sur le topic <code>/sensor_data</code>,</li>
                          <li>Définit un timer pour appeler <code>timer_callback</code> toutes les 0.5 secondes,</li>
                          <li>Affiche un message dans les logs ROS à l'initialisation.</li>
                        </ul>
                      </div>
                    </div>

                    {/* Bloc 4 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        4- Méthode <code>timer_callback</code>
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def timer_callback(self):
    temp = random.uniform(15.0, 35.0)
    hum = random.uniform(30.0, 70.0)
    pres = random.uniform(950.0, 1050.0)

    msg = Float32MultiArray()
    msg.data = [float(temp), float(hum), float(pres)]
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publié: temp={temp:.2f}°C, hum={hum:.2f}%, pres={pres:.2f}hPa')`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400">
                          <li>Génère 3 valeurs aléatoires représentant :
                            <ul className="list-['-_'] pl-5 mt-1">
                                <li>température (15–35 °C),</li>
                                <li>humidité (30–70 %),</li>
                                <li>pression (950–1050 hPa),</li>
                            </ul>
                          </li>
                          <li>Crée un message ROS <code>Float32MultiArray</code> avec ces 3 valeurs,</li>
                          <li>Publie le message,</li>
                          <li>Affiche les données publiées dans les logs ROS.</li>
                        </ul>
                      </div>
                    </div>

                    {/* Bloc 5 */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        5- Fonction <code>main</code>
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">python</span>
                        <pre className="text-sm text-gray-100"><code>{`def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('SensorPublisher arrêt.')
        node.destroy_node()
        rclpy.shutdown()`}</code></pre>
                      </div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        <p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p>
                        <ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400">
                          <li>Initialise le système ROS 2,</li>
                          <li>Lance le nœud <code>SensorPublisher</code>,</li>
                          <li>Reste actif (avec <code>rclpy.spin</code>) jusqu'à interruption (Ctrl+C),</li>
                          <li>Nettoie et ferme proprement le nœud ROS à l’arrêt.</li>
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
ros2 run sensor_data_evaluation sensor_publisher`}</code></pre>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-2/it/launch-configuration">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Launch Configuration
                  </Button>
                </Link>
                <Link href="/docs/semaine-2/it/sensor-subscriber">
                  <Button>
                    Sensor Subscriber
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