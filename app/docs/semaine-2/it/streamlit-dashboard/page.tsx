"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, BarChart2, PlayCircle, Columns, Settings, AlertCircle, Clock, LineChart, List, FileText, CheckCheck, Lock, Palette
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';
import { useMounted } from "@/hooks/use-mounted";

export default function StreamlitAppDocPage() {
  const mounted = useMounted();
  
  if (!mounted) {
    return null;
  }

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
                    <span>Interface Web – streamlit_app.py</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <BarChart2 className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Interface Web – <code>streamlit_app.py</code></h1>
                      <p className="text-muted-foreground">Dashboard en temps réel pour la visualisation de données de capteurs ROS2.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Python</Badge>
                    <Badge variant="outline">Streamlit</Badge>
                    <Badge variant="outline">Dashboard</Badge>
                   </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <PageNavigation />
          </div>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <AnimatedSection animation="fade-up">
                <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/general.png" alt="Vue générale du dashboard Streamlit" width={800} height={450} className="rounded-lg border border-gray-200 dark:border-gray-700 mx-auto" />
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
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
                        Cette application Streamlit sert de <strong>dashboard en temps réel</strong> pour la visualisation des données issues de capteurs ROS2 (température, humidité, pression).
                      </p>
                      <p className="mt-4">
                        Elle lit les données depuis le fichier <code>latest_sensor_data.json</code>, les affiche dynamiquement sous forme de <strong>cartes stylisées</strong>, <strong>graphiques évolutifs</strong>, <strong>tableaux d’historique</strong>, et <strong>statistiques</strong>.
                      </p>
                      <p className="mt-4">
                        Elle propose également des <strong>interactions via une sidebar</strong> permettant de contrôler l'affichage, les seuils d’alerte, et le rafraîchissement automatique.
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
                      <Columns className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Composants de l’interface utilisateur
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    
                    {/* Sidebar */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">1. <strong>Sidebar (panneau de contrôle)</strong></h3>
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                        <div className="space-y-4">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/affichage_historique.png" alt="Bouton Démarrer / Arrêter et Effacement de l’historique" width={200} height={150} className="rounded-md border border-gray-200 dark:border-gray-700" />
                          <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400">
                            <li>Bouton <strong>Démarrer / Arrêter</strong> l’affichage des nouvelles données.</li>
                            <li><strong>Effacement de l’historique</strong> en un clic.</li>
                          </ul>
                        </div>
                        <div className="space-y-4">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/seuil.png" alt="Seuils d’alerte" width={268} height={469} className="rounded-md border border-gray-200 dark:border-gray-700" />
                          <p className="text-sm text-gray-600 dark:text-gray-400"><strong>Configuration dynamique des seuils d’alerte</strong> (via sliders) pour chaque capteur : Température (°C), Humidité (%), Pression (hPa).</p>
                        </div>
                        <div className="space-y-4">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/actualisation.png" alt="Rafraîchissement" width={291} height={218} className="rounded-md border border-gray-200 dark:border-gray-700" />
                          <p className="text-sm text-gray-600 dark:text-gray-400">Options de <strong>rafraîchissement</strong> : Activation/désactivation, choix de l’intervalle (1s, 2s, 5s, 10s), et bouton “🔄 Actualiser maintenant”.</p>
                        </div>
                        <div className="space-y-4">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/etat_systeme.png" alt="État du système" width={192} height={181} className="rounded-md border border-gray-200 dark:border-gray-700" />
                          <p className="text-sm text-gray-600 dark:text-gray-400"><strong>État du système</strong> affiché en temps réel : Statut d’affichage, nombre de mesures, compteur de refreshs.</p>
                          <p className="text-sm text-gray-600 dark:text-gray-400 mt-2"><strong>Instructions rapides</strong> pour l’utilisateur.</p>
                        </div>
                      </div>
                    </div>

                    {/* Affichage temps réel */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">2. <strong>Affichage en temps réel (cartes métriques)</strong></h3>
                      <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">Trois cartes stylisées, avec couleurs et animations selon l’état : 🌡️ <strong>Température</strong>, 💧 <strong>Humidité</strong>, 🌪️ <strong>Pression</strong>. Chaque carte affiche la valeur actuelle, l'unité, une icône et l'état de conformité (`✅` ou `⚠️`).</p>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/dash_normal.png" alt="Affichage en temps réel / Normal" width={700} height={200} className="rounded-md border border-gray-200 dark:border-gray-700 mb-4" />
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm italic">
                        <p className="text-gray-600 dark:text-gray-400">💡 En cas de dépassement des seuils, la carte clignote et un message d'erreur s’affiche.</p>
                      </div>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/dash_pb.png" alt="Affichage en temps réel / Hors plage" width={700} height={200} className="rounded-md border border-gray-200 dark:border-gray-700 mt-4" />
                    </div>

                    {/* Alertes */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg flex items-center gap-2"><AlertCircle className="w-5 h-5" /> 3. <strong>Alertes</strong></h3>
                      <p className="text-sm text-gray-600 dark:text-gray-400">Deux types d’alertes sont affichées :</p>
                      <ul className="list-disc pl-5 mt-2 space-y-1 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>Alerte Dashboard</strong> : basée sur les seuils configurés dans la sidebar.</li>
                        <li><strong>Alerte ROS2</strong> : basée sur les seuils du <code>sensor_subscriber.py</code> (fichier JSON).</li>
                      </ul>
                    </div>

                    {/* Dernière mise à jour */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg flex items-center gap-2"><Clock className="w-5 h-5" /> 4. <strong>Dernière mise à jour</strong></h3>
                      <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">Un indicateur donne la <strong>date et l’heure</strong> de la dernière mesure reçue et affichée.</p>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/update_time.png" alt="Dernière mise à jour" width={400} height={50} className="rounded-md border border-gray-200 dark:border-gray-700" />
                    </div>
                    
                    {/* Onglets dynamiques */}
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-4 text-lg">5. <strong>Onglets dynamiques</strong></h3>
                      <div className="space-y-6">
                        <div>
                          <h4 className="font-semibold text-gray-800 dark:text-gray-200 mb-2 flex items-center gap-2"><LineChart className="w-4 h-4" /> 📈 <strong>Graphiques (Plotly)</strong></h4>
                          <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400 space-y-1 mb-4">
                            <li>3 graphiques synchronisés (température, humidité, pression)</li>
                            <li>Affichage en ligne + points</li>
                            <li>Données historisées jusqu’à 100 mesures</li>
                            <li>Graphiques interactifs : zoom, survol, export</li>
                          </ul>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/plotly.png" alt="Graphiques Plotly" width={700} height={300} className="rounded-md border border-gray-200 dark:border-gray-700" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-800 dark:text-gray-200 mb-2 flex items-center gap-2"><List className="w-4 h-4" /> 📋 <strong>Historique</strong></h4>
                          <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400 space-y-1 mb-4">
                            <li>Tableau des <strong>20 dernières mesures</strong></li>
                            <li>Affichage formaté : arrondi à 2 décimales, horodatage simplifié</li>
                            <li>Présence d’indicateurs de validation (<code>temp_ok</code>, <code>hum_ok</code>, <code>pres_ok</code>)</li>
                          </ul>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/history_20.png" alt="Historique des 20 dernières mesures" width={700} height={300} className="rounded-md border border-gray-200 dark:border-gray-700" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-800 dark:text-gray-200 mb-2 flex items-center gap-2"><BarChart2 className="w-4 h-4" /> 📊 <strong>Statistiques</strong></h4>
                          <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400 space-y-1 mb-4">
                            <li>Moyenne, min et max pour chaque capteur</li>
                            <li>Affichage clair dans 3 colonnes</li>
                            <li>Mise à jour automatique à chaque ajout de donnée</li>
                          </ul>
                          <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/IT/media/stat.png" alt="Statistiques" width={700} height={150} className="rounded-md border border-gray-200 dark:border-gray-700" />
                        </div>
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
                      <Settings className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Fonctionnalités techniques avancées
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-4 text-gray-700 dark:text-gray-300">
                      <li>
                        <strong>Auto-refresh intelligent</strong> (via <code>streamlit_autorefresh</code>) avec synchronisation sur l’état <code>running</code>.
                      </li>
                      <li>
                        <strong>Sécurité des données</strong> :
                        <ul className="list-['-_'] pl-5 mt-1 text-sm text-gray-600 dark:text-gray-400">
                          <li>Vérification de la structure JSON</li>
                          <li>Lecture sécurisée (try/except)</li>
                        </ul>
                      </li>
                      <li>
                        <strong>Affichage stylisé</strong> :
                        <ul className="list-['-_'] pl-5 mt-1 text-sm text-gray-600 dark:text-gray-400">
                          <li>Utilisation de <strong>CSS custom</strong> pour les composants</li>
                          <li>Animation <code>pulse</code> pour signaler un danger</li>
                        </ul>
                      </li>
                      <li>
                        <strong>Session persistante</strong> :
                        <ul className="list-['-_'] pl-5 mt-1 text-sm text-gray-600 dark:text-gray-400">
                          <li>Historique en mémoire dans <code>st.session_state</code></li>
                          <li>Configuration utilisateur conservée au rafraîchissement</li>
                        </ul>
                      </li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Instructions d’utilisation
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-4 text-gray-700 dark:text-gray-300">
                      <li>
                        Lancer les scripts ROS 2 :
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-2 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                          <pre className="text-sm text-gray-100"><code>{`ros2 run sensor_data_evaluation sensor_publisher
ros2 run sensor_data_evaluation sensor_subscriber`}</code></pre>
                        </div>
                      </li>
                      <li>
                        Démarrer le dashboard Streamlit :
                        <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 my-2 overflow-x-auto relative border border-gray-700">
                          <span className="absolute top-2 right-2 text-xs text-gray-400">bash</span>
                          <pre className="text-sm text-gray-100"><code>{`streamlit run streamlit_app.py`}</code></pre>
                        </div>
                      </li>
                      <li>
                        Ajuster les seuils et visualiser les données en temps réel !
                      </li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-2/it/sensor-subscriber">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Sensor Subscriber
                  </Button>
                </Link>
                <Link href="/docs/semaine-2">
                  <Button>
                    Retour Semaine 2
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