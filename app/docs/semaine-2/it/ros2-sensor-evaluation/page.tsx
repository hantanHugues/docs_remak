"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, GitBranch, PlayCircle, FolderTree, BarChart2, Upload, Download, Rocket
} from "lucide-react";
import Link from "next/link";

export default function Ros2SensorEvalPage() {
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
                    <span>Évaluation de Données Capteur</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <GitBranch className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">ROS2 Sensor Data Evaluation</h1>
                      <p className="text-muted-foreground">Mise en pratique de ROS2 avec un système publisher-subscriber et une interface Streamlit.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">ROS 2</Badge>
                    <Badge variant="outline">Python</Badge>
                    <Badge variant="outline">Streamlit</Badge>
                    <Badge variant="outline">Projet</Badge>
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
                      Objectif du Test
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Ce test a pour objectif de mettre en pratique ROS2 (en Python) en créant un package <code>sensor_data_evaluation</code> qui contient :
                      </p>
                      <ul className="list-disc pl-5 my-4 space-y-2 text-sm">
                        <li>Un <strong>node publisher</strong> qui génère des données aléatoires de capteurs (température, humidité, pression) et publie sur un topic ROS2.</li>
                        <li>Un <strong>node subscriber</strong> qui reçoit ces données, vérifie qu’elles sont dans les plages attendues, loggue les éventuelles alertes, et écrit la dernière mesure dans un fichier JSON.</li>
                        <li>Une <strong>application Streamlit</strong> qui lit ce fichier JSON et affiche un dashboard en temps réel des mesures (métriques, historique, graphiques, statistiques, alertes).</li>
                      </ul>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FolderTree className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Structure du projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                      <pre className="text-sm text-gray-100 font-mono"><code>{`Test2/
└── src/
    └── sensor_data_evaluation/
        ├── launch/
        │   └── sensor_launch.py         # Lance publisher et subscriber ensemble.
        ├── resource/
        ├── sensor_data_evaluation/      # Package principal
        │   ├── sensor_publisher.py      # Node ROS2 qui publie des données aléatoires.
        │   ├── sensor_subscriber.py     # Node ROS2 qui vérifie les données reçues et les sauvegarde dans un JSON.
        │   └── streamlit_app.py         # Interface Streamlit qui lit les données JSON et les affiche.
        ├── test/
        ├── package.xml
        ├── setup.cfg
        └── setup.py`}</code></pre>
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
                      Compilation et exécution rapide
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    
                    {/* Compilation */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        1. Compilation
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                        <pre className="text-sm text-gray-100"><code>{`colcon build --packages-select sensor_data_evaluation`}</code></pre>
                      </div>
                    </div>

                    {/* Sourcing */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        2. Sourcing
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                        <pre className="text-sm text-gray-100"><code>{`source install/setup.bash`}</code></pre>
                      </div>
                    </div>

                    {/* Exécution séparée */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        3. Exécution séparée
                      </h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400 mb-2"><strong>Publisher :</strong> Dans Terminal 1</p>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                        <pre className="text-sm text-gray-100"><code>{`ros2 run sensor_data_evaluation sensor_publisher`}</code></pre>
                      </div>
                      <p className="text-sm text-gray-600 dark:text-gray-400 mb-2"><strong>Subscriber :</strong> Dans Terminal 2</p>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                        <pre className="text-sm text-gray-100"><code>{`ros2 run sensor_data_evaluation sensor_subscriber`}</code></pre>
                      </div>
                    </div>

                    {/* Exécution via launch */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        4. Exécution via launch
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                        <pre className="text-sm text-gray-100"><code>{`ros2 launch sensor_data_evaluation sensor_launch.py`}</code></pre>
                      </div>
                    </div>

                    {/* Lancement Streamlit */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">
                        5. Lancement de l'interface Streamlit
                      </h4>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 overflow-x-auto relative border border-gray-700">
                        <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                        <pre className="text-sm text-gray-100"><code>{`cd src/sensor_data_evaluation/sensor_data_evaluation/
streamlit run streamlit_app.py`}</code></pre>
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
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Documentation Détaillée
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="grid md:grid-cols-2 gap-4">
                      <Link href="Documentation/semaine-2/IT/readme_P.md" className="group flex items-center gap-3 p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors border border-gray-200 dark:border-gray-700">
                        <Upload className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        <div>
                          <p className="font-semibold text-gray-900 dark:text-gray-100">sensor_publisher</p>
                          <p className="text-xs text-gray-500 dark:text-gray-400">Génère et publie les données des capteurs</p>
                        </div>
                      </Link>
                      <Link href="Documentation/semaine-2/IT/readme_S.md" className="group flex items-center gap-3 p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors border border-gray-200 dark:border-gray-700">
                        <Download className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        <div>
                          <p className="font-semibold text-gray-900 dark:text-gray-100">sensor_subscriber</p>
                          <p className="text-xs text-gray-500 dark:text-gray-400">Traite et analyse les données reçues</p>
                        </div>
                      </Link>
                      <Link href="Documentation/semaine-2/IT/readme_launch.md" className="group flex items-center gap-3 p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors border border-gray-200 dark:border-gray-700">
                        <Rocket className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        <div>
                          <p className="font-semibold text-gray-900 dark:text-gray-100">sensor_launch</p>
                          <p className="text-xs text-gray-500 dark:text-gray-400">Script de lancement simultané des nodes</p>
                        </div>
                      </Link>
                      <Link href="Documentation/semaine-2/IT/readme_st.md" className="group flex items-center gap-3 p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors border border-gray-200 dark:border-gray-700">
                        <BarChart2 className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        <div>
                          <p className="font-semibold text-gray-900 dark:text-gray-100">app streamlit</p>
                          <p className="text-xs text-gray-500 dark:text-gray-400">Interface graphique de visualisation</p>
                        </div>
                      </Link>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-2/it">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Section IT
                  </Button>
                </Link>
                <Link href="/docs/semaine-2/it/launch-configuration">
                  <Button>
                    Launch Configuration
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