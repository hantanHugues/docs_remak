"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, TrendingUp, Cpu, Monitor, ShieldCheck, Database, Layers, Zap, Shield, LayoutGrid, HardDrive, Wifi, Bell, CheckSquare, GitCompareArrows, Box, CloudUpload
} from "lucide-react";
import Link from "next/link";
import { useMounted } from "@/hooks/use-mounted";


export default function ImprovementsGuideFRPage() {
  const mounted = useMounted();
  
  if (!mounted) {
    return null;
  }

  // Helper pour créer des IDs valides pour les ancres
  const slugify = (text: string) =>
    text
      .toLowerCase()
      .replace(/[^\w\s-]/g, '')
      .replace(/[\s_-]+/g, '-')
      .replace(/^-+|-+$/g, '');

  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-green-50 via-white to-teal-50 dark:from-green-950/20 dark:via-background dark:to-teal-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Améliorations Possibles</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <TrendingUp className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Améliorations Possibles - Boîte Noire & Station de Contrôle</h1>
                      <p className="text-muted-foreground">Pistes d'amélioration pour les capacités, la robustesse et l'ergonomie du système.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Évolution</Badge>
                    <Badge variant="outline">Conception</Badge>
                    <Badge variant="outline">Électronique</Badge>
                    <Badge variant="outline">Logiciel</Badge>
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
            <div className="max-w-5xl mx-auto">
              
              <AnimatedSection animation="fade-up">
                <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-8 mb-12">
                  <p className="text-center text-gray-700 dark:text-gray-300 leading-relaxed">
                    Le système "Boîte Noire & Station de Contrôle" est fonctionnel et remplit ses objectifs initiaux. Cependant, plusieurs pistes d'amélioration peuvent être explorées pour enrichir ses capacités, sa robustesse et sa convivialité.
                  </p>
                </div>
              </AnimatedSection>

              <Separator className="mb-12" />

              {/* SECTION 1 - Améliorations de la Boîte Noire */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-gray-700 mt-12">
                  <CardHeader className="bg-gradient-to-r from-gray-50 to-transparent dark:from-gray-950/20">
                    <CardTitle id={slugify("1. Améliorations du Module Boîte Noire")} className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <Cpu className="w-6 h-6 text-gray-700" />
                      1. Améliorations du Module Boîte Noire
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Database className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Gestion Avancée de l'EEPROM</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Implémenter une gestion plus sophistiquée du tampon circulaire de l'EEPROM pour garantir l'intégrité des données même en cas de coupure de courant pendant l'écriture, ou pour ajouter des horodatages aux enregistrements.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Layers className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Acquisition de Données Supplémentaires</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Intégrer d'autres capteurs (GPS pour la localisation du crash, baromètre pour l'altitude, capteur de température/humidité) pour enrichir les données de vol.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Zap className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Optimisation de la Consommation d'Énergie</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Mettre en œuvre des modes basse consommation pour prolonger l'autonomie de la batterie de la Boîte Noire, ce qui est essentiel pour les applications embarquées.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Shield className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Précision de la Détection de Crash</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Affiner l'algorithme de détection de crash en utilisant des techniques de filtrage (filtre de Kalman, filtre complémentaire) pour le MPU-6050, ou en intégrant des seuils dynamiques.
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 2 - Améliorations de la Station de Contrôle */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-green-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-green-50 to-transparent dark:from-green-950/20">
                    <CardTitle id={slugify("2. Améliorations du Module Station de Contrôle")} className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <Monitor className="w-6 h-6 text-green-600" />
                      2. Améliorations du Module Station de Contrôle
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <LayoutGrid className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Interface Utilisateur Enrichie</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Remplacer l'écran LCD par un écran graphique OLED ou TFT pour un affichage plus riche (graphiques, menus intuitifs, icônes).
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <HardDrive className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Stockage des Données Reçues</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Ajouter une carte SD ou une EEPROM externe pour que la Station de Contrôle puisse enregistrer un historique des données reçues en temps réel, et pas seulement après un crash.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Wifi className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Connectivité Étendue</h4>
                          <ul className="list-disc pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <li>Intégration d'un module Wi-Fi (ESP8266/ESP32) ou Bluetooth pour une communication sans fil avec un PC ou un smartphone, permettant une visualisation plus avancée ou une transmission à distance.</li>
                            <li>Mise en place d'un serveur web embarqué sur l'ESP32 pour visualiser les données via un navigateur.</li>
                          </ul>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Bell className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Alertes Audio/Visuelles</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Ajouter un buzzer ou des LEDs pour des alertes plus visibles ou audibles en cas de crash.
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* SECTION 3 - Améliorations de la Communication et Robustesse */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-orange-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-orange-50 to-transparent dark:from-orange-950/20">
                    <CardTitle id={slugify("3. Améliorations de la Communication et de la Robustesse")} className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <ShieldCheck className="w-6 h-6 text-orange-600" />
                      3. Améliorations de la Communication et de la Robustesse
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <CheckSquare className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Checksum/CRC pour I2C</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Ajouter un mécanisme de vérification d'erreur (somme de contrôle ou CRC) aux paquets I2C pour garantir l'intégrité des données transmises.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <GitCompareArrows className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Redondance des Capteurs</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Pour les applications critiques, envisager d'utiliser plusieurs capteurs et une logique de fusion de données pour une plus grande fiabilité.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <Box className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Boîtier et Montage</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Concevoir et imprimer en 3D un boîtier robuste et compact pour chaque module, protégeant l'électronique et facilitant l'intégration dans un drone/robot.
                          </p>
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <div className="flex items-start gap-4">
                        <div className="w-10 h-10 bg-gray-100 dark:bg-gray-800 rounded-lg flex items-center justify-center flex-shrink-0">
                          <CloudUpload className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        </div>
                        <div>
                          <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Mises à Jour FOTA (Firmware Over-The-Air)</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Si la connectivité Wi-Fi est ajoutée, permettre les mises à jour du firmware à distance pour faciliter la maintenance et l'évolution.
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 mt-12 border-t">
                <Link href="#">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Page Précédente
                  </Button>
                </Link>
                <Link href="#">
                  <Button>
                    Page Suivante
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