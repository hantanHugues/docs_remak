"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, Workflow, PlayCircle, Download, CheckSquare, AlertTriangle, Save
} from "lucide-react";
import Link from "next/link";

export default function SensorSubscriberDocPage() {
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
                    <span>sensor_subscriber.py</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Download className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold"><code>sensor_subscriber.py</code></h1>
                      <p className="text-muted-foreground">Nœud ROS 2 pour la réception, la validation et la sauvegarde de données de capteur.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">ROS 2</Badge>
                    <Badge variant="outline">Subscriber</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-2/it/sensor-publisher">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Sensor Publisher
                  </Button>
                </Link>
                <Link href="/docs/semaine-2/it/streamlit-dashboard">
                  <Button variant="ghost" size="sm">
                    Streamlit Dashboard
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
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Description générale
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Ce script définit un <strong>nœud ROS 2</strong> nommé <code>sensor_subscriber</code> qui <strong>écoute</strong> les messages publiés sur le topic <code>/sensor_data</code>.
                      </p>
                      <p className="mt-4">
                        Il lit les mesures de température, d'humidité et de pression, vérifie si elles sont dans des <strong>plages prédéfinies</strong>, affiche des <strong>logs d'alerte</strong> si nécessaire, puis <strong>enregistre les dernières valeurs reçues</strong> dans un fichier JSON (<code>latest_sensor_data.json</code>).
                      </p>
                      <p className="mt-4 bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">
                        Ce fichier sert ensuite d’entrée à l’application <strong>Streamlit</strong> pour l’affichage des données en temps réel.
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
                      Analyse de chaque partie du script du subscriber.
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    
                    {/* Blocs de code ici */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">1- Importations</h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
from datetime import datetime`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction : Importe les modules nécessaires</p><ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400"><li><code>rclpy</code> & <code>Node</code> : pour créer un nœud ROS 2,</li><li><code>Float32MultiArray</code> : type de message ROS attendu,</li><li><code>json</code> : pour sauvegarder les données vers un fichier JSON,</li><li><code>datetime</code> : pour ajouter un timestamp ISO aux données.</li></ul></div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">2- Classe <code>SensorSubscriber</code></h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`class SensorSubscriber(Node):`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p><p className="text-gray-600 dark:text-gray-400">Définit un nœud abonné au topic <code>/sensor_data</code>.</p></div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">3- Constructeur <code>__init__</code></h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`def __init__(self):
    super().__init__('sensor_subscriber')
    self.subscription = self.create_subscription(
        Float32MultiArray,
        '/sensor_data',
        self.listener_callback,
        10)
    ...`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p><ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400"><li>Crée le nœud ROS nommé <code>sensor_subscriber</code>,</li><li>Crée un abonnement au topic <code>/sensor_data</code>,</li><li>Définit les <strong>plages valides</strong> pour chaque mesure :<ul className="list-['-_'] pl-5 mt-1"><li>Température : 15.0–35.0 °C</li><li>Humidité : 30.0–70.0 %</li><li>Pression : 950.0–1050.0 hPa</li></ul></li></ul></div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">4- Méthode <code>listener_callback</code></h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`def listener_callback(self, msg: Float32MultiArray):`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction : Fonction appelée à chaque message reçu. Elle :</p><ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400"><li>Vérifie que le message contient bien <strong>3 éléments</strong> (temp, hum, pres).</li><li>Vérifie si chaque mesure est dans sa plage normale.</li><li>Affiche des <strong>logs d’erreur</strong> si l’une des valeurs est hors plage.</li><li>Crée un dictionnaire structuré avec les données et un <strong>timestamp ISO</strong>.</li><li>Enregistre ce dictionnaire dans un fichier JSON (<code>latest_sensor_data.json</code>).</li></ul></div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 flex items-center gap-2"><CheckSquare className="w-4 h-4" /> 5- Vérification des valeurs</h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`temp_ok = self.temp_range[0] <= temp <= self.temp_range[1]
hum_ok = self.hum_range[0] <= hum <= self.hum_range[1]
pres_ok = self.pres_range[0] <= pres <= self.pres_range[1]`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p><p className="text-gray-600 dark:text-gray-400">Vérifie que chaque mesure se situe dans les plages prédéfinies.</p></div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 flex items-center gap-2"><Save className="w-4 h-4" /> 6- Sauvegarde JSON</h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`sensor_data = {
    'temperature': float(temp),
    'humidity': float(hum),
    'pressure': float(pres),
    'timestamp': datetime.now().isoformat(),
    'temp_ok': temp_ok,
    'hum_ok': hum_ok,
    'pres_ok': pres_ok
}
with open('latest_sensor_data.json', 'w') as f:
    json.dump(sensor_data, f, indent=2)`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p><ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400"><li>Structure les données avec leur statut (<code>ok</code> ou non),</li><li>Ajoute l’heure d’acquisition,</li><li>Écrit le tout dans un fichier JSON utilisé par Streamlit.</li></ul></div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3 flex items-center gap-2"><AlertTriangle className="w-4 h-4" /> 7- Gestion des erreurs</h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`except Exception as e:
    self.get_logger().warning(f'Erreur sauvegarde: {e}')`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p><p className="text-gray-600 dark:text-gray-400">Si la sauvegarde du fichier échoue (ex: permissions, espace disque), un avertissement est affiché.</p></div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">8- Fonction <code>main</code></h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    ...`}</code></pre></div>
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm"><p className="font-semibold text-gray-700 dark:text-gray-300 mb-2">Fonction :</p><ul className="list-disc pl-5 space-y-1 text-gray-600 dark:text-gray-400"><li>Initialise ROS 2,</li><li>Instancie et lance le nœud <code>SensorSubscriber</code>,</li><li>Reste actif en attente de messages,</li><li>Libère les ressources proprement à la fermeture (Ctrl+C).</li></ul></div>
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
ros2 run sensor_data_evaluation sensor_subscriber`}</code></pre>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-2/it/sensor-publisher">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Sensor Publisher
                  </Button>
                </Link>
                <Link href="/docs/semaine-2/it/streamlit-dashboard">
                  <Button>
                    Streamlit Dashboard
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