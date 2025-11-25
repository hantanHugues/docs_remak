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
  Target, ArrowLeft, ArrowRight, BookOpen, Cpu, Wrench, CircuitBoard, GitBranch, Code, PlayCircle,
  Lightbulb, CheckCircle, ListChecks, FileText, AlertTriangle, DownloadCloud, Users, Calendar,
  School, Zap, Settings, Monitor, Database, Workflow, TrendingUp, Shield, ExternalLink
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';
import { useMounted } from "@/hooks/use-mounted";

export default function TekbotChallengeGyroscopePage() {
  const mounted = useMounted();
  
  if (!mounted) {
    return null;
  }

  // Helper pour créer des IDs valides pour les ancres
  const slugify = (text: string) =>
    text
      .toLowerCase()
      .replace(/[^\w\s-]/g, '') // remove non-word chars
      .replace(/[\s_-]+/g, '-') // collapse whitespace and replace _ with -
      .replace(/^-+|-+$/g, ''); // remove leading/trailing dashes

  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Header */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-blue-50 via-white to-cyan-50 dark:from-blue-950/20 dark:via-background dark:to-cyan-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Tekbot Challenge 2025
                    </Link>
                    <span>/</span>
                    <span>Épreuve Électronique</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <CircuitBoard className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Test 1 : Gyroscope & Accéléromètre (MPU-6050)</h1>
                      <p className="text-muted-foreground">Gestion Durable des Déchets – Épreuve Électronique</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Projet</Badge>
                    <Badge variant="outline">Électronique</Badge>
                    <Badge variant="outline">Arduino</Badge>
                    <Badge variant="outline">Capteurs</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <PageNavigation />

          {/* Content */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-5xl mx-auto">
              
              {/* Équipe et infos projet */}
              <AnimatedSection animation="fade-up">
                <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-8 mb-12">
                  <div className="grid md:grid-cols-2 gap-8">
                    <div>
                      <h2 className="text-xl font-semibold mb-6 text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        Équipe IFRI Électronique
                      </h2>
                      <div className="space-y-3">
                        {[
                          "Aretha FAGLA",
                          "Hugues HANTAN", 
                          "Marielle AGBOSSOUNON",
                          "Eunice ODJO",
                          "Livingstone GBOZO"
                        ].map((name, index) => (
                          <div key={index} className="flex items-center gap-3">
                            <div className="w-1.5 h-1.5 bg-gray-400 dark:bg-gray-500 rounded-full flex-shrink-0"></div>
                            <span className="text-sm text-gray-700 dark:text-gray-300">{name}</span>
                          </div>
                        ))}
                      </div>
                    </div>
                    
                    <div>
                      <h2 className="text-xl font-semibold mb-6 text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        Informations du projet
                      </h2>
                      <div className="space-y-4">
                        <div>
                          <div className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Institution</div>
                          <div className="text-sm text-gray-600 dark:text-gray-400">
                            Institut de Formation et de Recherche en Informatique (IFRI)<br/>
                            Université d'Abomey-Calavi
                          </div>
                        </div>
                        <div>
                          <div className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Date de réalisation</div>
                          <div className="text-sm text-gray-600 dark:text-gray-400">12 Juin 2025</div>
                        </div>
                        <div>
                          <div className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">Compétition</div>
                          <div className="text-sm text-gray-600 dark:text-gray-400">Tekbot Robotics Challenge 2025</div>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </AnimatedSection>
              
              {/* Table des matières */}
              <AnimatedSection animation="fade-up" delay={50}>
                <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6 mb-12">
                  <h2 className="text-2xl font-semibold mb-6 text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-3">
                    Table des matières
                  </h2>
                  <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-3">
                    {[
                      { icon: Target, title: "Introduction et Objectifs", href: "#introduction" },
                      { icon: Cpu, title: "Matériel et branchements", href: "#materiel" },
                      { icon: Lightbulb, title: "Principe du MPU-6050", href: "#principe" },
                      { icon: GitBranch, title: "Architecture du système", href: "#architecture" },
                      { icon: CircuitBoard, title: "Schéma électrique", href: "#schema" },
                      { icon: Code, title: "Installation du code", href: "#installation" },
                      { icon: FileText, title: "Explication du code", href: "#code" },
                      { icon: PlayCircle, title: "Mode d'emploi", href: "#emploi" },
                      { icon: TrendingUp, title: "Résultats attendus", href: "#resultats" },
                      { icon: Settings, title: "Calibration", href: "#calibration" },
                      { icon: AlertTriangle, title: "Limitations", href: "#limitations" },
                      { icon: BookOpen, title: "Références", href: "#references" }
                    ].map((item, index) => {
                      const IconComponent = item.icon;
                      return (
                        <a key={index} href={item.href} className="group flex items-center gap-3 p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg hover:bg-gray-100 dark:hover:bg-gray-800 transition-all duration-200 border border-gray-200 dark:border-gray-700">
                          <div className="w-8 h-8 bg-gray-200 dark:bg-gray-700 rounded-lg flex items-center justify-center">
                            <IconComponent className="w-4 h-4 text-gray-600 dark:text-gray-400" />
                          </div>
                          <span className="text-sm font-medium text-gray-700 dark:text-gray-300 group-hover:text-gray-900 dark:group-hover:text-gray-100">{item.title}</span>
                        </a>
                      );
                    })}
                  </div>
                </div>
              </AnimatedSection>
              
              <Separator />

              {/* SECTION 1 - Introduction et Objectifs */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-blue-500">
                  <CardHeader className="bg-gradient-to-r from-blue-50 to-transparent dark:from-blue-950/20">
                    <CardTitle id="1-introduction-et-objectifs" className="flex items-center gap-2 scroll-mt-24 text-xl">
                      <Target className="w-6 h-6 text-blue-600" />
                      1. Introduction et Objectifs
                    </CardTitle>
                    <CardDescription>
                      Contexte du projet et objectifs techniques à atteindre
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6">
                    {/* Contexte */}
                    <div className="space-y-4">
                      <div className="flex items-center gap-2 mb-3">
                        <div className="w-8 h-8 rounded-lg bg-gray-100 dark:bg-gray-800 flex items-center justify-center">
                          <span className="text-gray-600 font-semibold text-sm">1.1</span>
                        </div>
                        <h3 className="text-lg font-semibold">Contexte</h3>
                      </div>
                      
                      <div className="bg-gradient-to-r from-blue-50 to-cyan-50 dark:from-blue-950/10 dark:to-cyan-950/10 p-4 rounded-lg border border-blue-200 dark:border-blue-800">
                        <p className="text-sm leading-relaxed">
                          Le <strong>TEKBOT Robotics Challenge 2025</strong> est une compétition internationale annuelle, organisée par TEKBOT Robotics (startup DeepTech basée au Bénin), qui met en compétition de jeunes talents africains autour de défis robotiques et d'IA.
                        </p>
                      </div>
                      
                      <div className="grid md:grid-cols-2 gap-4">
                        <div className="p-4 bg-muted/30 rounded-lg">
                          <h4 className="font-semibold mb-2 flex items-center gap-2">
                            <Zap className="w-4 h-4 text-gray-500" />
                            Thème 2025
                          </h4>
                          <p className="text-sm text-muted-foreground">« Résilience Urbaine : Gestion Durable des Déchets »</p>
                        </div>
                        <div className="p-4 bg-muted/30 rounded-lg">
                          <h4 className="font-semibold mb-2 flex items-center gap-2">
                            <Monitor className="w-4 h-4 text-gray-500" />
                            Objectif
                          </h4>
                          <p className="text-sm text-muted-foreground">Robots autonomes de collecte et tri de déchets dans EcoCity</p>
                        </div>
                      </div>
                      
                      <div className="bg-gray-50 dark:bg-gray-900 border border-gray-200 dark:border-gray-700 p-4 rounded-lg">
                        <h4 className="font-semibold mb-2 text-gray-800 dark:text-gray-200">Mesures requises pour le Test 1</h4>
                        <div className="grid md:grid-cols-3 gap-3">
                          {[
                            { icon: "↕️", text: "Orientation du mouvement" },
                            { icon: "⚡", text: "Vitesse du mouvement" },
                            { icon: "📺", text: "Affichage LCD 16×2 temps réel" }
                          ].map((item, index) => (
                            <div key={index} className="flex items-center gap-2 text-sm">
                              <span className="text-lg">{item.icon}</span>
                              <span>{item.text}</span>
                            </div>
                          ))}
                        </div>
                      </div>
                    </div>
                    
                    {/* Objectifs détaillés */}
                    <div className="space-y-6">
                      <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        1.2 Objectifs détaillés
                      </h3>
                      
                      <div className="space-y-4">
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Communication I2C</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Implémentation de la communication I²C pour la lecture en temps réel des registres du MPU-6050.
                          </p>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Calibration du capteur</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Mesure et correction du biais du gyroscope et établissement de la référence de l'accéléromètre.
                          </p>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Traitement des données</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Conversion des valeurs LSB vers les unités physiques réelles (g, °/s, m/s²) et application 
                            du filtrage numérique passe-bas (DLPF) avec fusion des données par filtre complémentaire.
                          </p>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Affichage des résultats</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Restitution en temps réel de l'orientation statique et de la vitesse d'accélération sur écran LCD 16×2.
                          </p>
                        </div>
                      </div>
                      
                      {/* Extension - Types de mouvements */}
                      <div className="mt-6">
                        <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Extension : Détection de mouvements dynamiques</h4>
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                            Dans le contexte d'une simulation de vol, le système détecte et classifie les types de mouvements suivants :
                          </p>
                          <div className="grid grid-cols-2 md:grid-cols-3 gap-2 text-sm">
                            {[
                              "Montée / descente",
                              "Virage gauche / droite", 
                              "Glissement avant / arrière",
                              "Glissement latéral",
                              "Vol stable",
                              "Mesure du Jerk"
                            ].map((movement, index) => (
                              <div key={index} className="flex items-center gap-2 text-gray-700 dark:text-gray-300">
                                <span className="text-xs">•</span>
                                <span>{movement}</span>
                              </div>
                            ))}
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 2 - Matériel et branchements */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle id="2-matériel-et-branchements" className="scroll-mt-24 text-xl text-gray-900 dark:text-gray-100">
                      2. Matériel et branchements
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Composants électroniques et schémas de connexion
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="prose prose-gray dark:prose-invert max-w-none">
                    <div className="space-y-8">
                      <h3 id={slugify("2.1 Composants électroniques")} className="text-2xl font-semibold mb-8 text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-3">
                        2.1 Composants électroniques
                      </h3>
                      
                      {/* Liste des composants avec style académique */}
                      <div className="space-y-8">
                        {/* Arduino Uno */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                          <div className="p-6">
                            <div className="flex items-start gap-6">
                              <div className="flex-shrink-0">
                                <div className="w-32 h-24 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/Arduino_Uno_-_R3.jpg" alt="Arduino Uno" width={128} height={96} className="w-full h-full object-cover" />
                                </div>
                              </div>
                              <div className="flex-1 space-y-3">
                                <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100">Arduino Uno</h4>
                                <div className="space-y-2">
                                  <div>
                                    <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Description : </span>
                                    <span className="text-sm text-gray-600 dark:text-gray-400">Microcontrôleur ATmega328P, 14 GPIO, 6 entrées analogiques.</span>
                                  </div>
                                  <div>
                                    <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Fonction dans le montage : </span>
                                    <span className="text-sm text-gray-600 dark:text-gray-400">Cœur du système responsable de la lecture du capteur, de l'exécution du code de traitement et du pilotage de l'affichage.</span>
                                  </div>
                                </div>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* MPU-6050 */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                          <div className="p-6">
                            <div className="flex items-start gap-6">
                              <div className="flex-shrink-0">
                                <div className="w-32 h-24 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/MPU6050.jpg" alt="MPU6050" width={128} height={96} className="w-full h-full object-cover" />
                                </div>
                              </div>
                              <div className="flex-1 space-y-3">
                                <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100">MPU-6050</h4>
                                <div className="space-y-2">
                                  <div>
                                    <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Description : </span>
                                    <span className="text-sm text-gray-600 dark:text-gray-400">Module I²C intégrant un accéléromètre 3 axes (±2g) et un gyroscope 3 axes (±250°/s).</span>
                                  </div>
                                  <div>
                                    <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Fonction dans le montage : </span>
                                    <span className="text-sm text-gray-600 dark:text-gray-400">Acquisition des données d'orientation et d'accélération selon les trois axes spatiaux.</span>
                                  </div>
                                </div>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Écran LCD */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                          <div className="p-6">
                            <div className="flex items-start gap-6">
                              <div className="flex-shrink-0">
                                <div className="w-32 h-24 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/LCD_Screen.jpg" alt="Écran LCD" width={128} height={96} className="w-full h-full object-cover" />
                                </div>
                              </div>
                              <div className="flex-1 space-y-3">
                                <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100">Écran LCD HD44780 16×2</h4>
                                <div className="space-y-2">
                                  <div>
                                    <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Description : </span>
                                    <span className="text-sm text-gray-600 dark:text-gray-400">Afficheur alphanumérique 16 caractères × 2 lignes, interface 4 bits.</span>
                                  </div>
                                  <div>
                                    <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Fonction dans le montage : </span>
                                    <span className="text-sm text-gray-600 dark:text-gray-400">Restitution en temps réel des données d'orientation et d'intensité du mouvement.</span>
                                  </div>
                                </div>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Composants auxiliaires */}
                        <div className="grid md:grid-cols-2 gap-6">
                          {/* Potentiomètre */}
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-center gap-4">
                              <div className="w-16 h-12 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded overflow-hidden flex-shrink-0">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/Pot_10K.jpg" alt="Potentiomètre 10kΩ" width={64} height={48} className="w-full h-full object-cover" />
                              </div>
                              <div className="flex-1">
                                <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-1">Potentiomètre 10kΩ</h5>
                                <p className="text-xs text-gray-600 dark:text-gray-400">Ajustement du contraste de l'écran LCD</p>
                              </div>
                            </div>
                          </div>

                          {/* Pile 9V */}
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-center gap-4">
                              <div className="w-16 h-12 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded overflow-hidden flex-shrink-0">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/Pile_9v.jpg" alt="Pile 9V" width={64} height={48} className="w-full h-full object-cover" />
                              </div>
                              <div className="flex-1">
                                <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-1">Pile rechargeable 9V</h5>
                                <p className="text-xs text-gray-600 dark:text-gray-400">Alimentation du système complet</p>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Matériel de prototypage */}
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Matériel de prototypage</h5>
                          <div className="flex items-center gap-4">
                            <div className="flex gap-2">
                              <div className="w-12 h-8 bg-white dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded overflow-hidden">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/breadboard.jpeg" alt="Breadboard" width={48} height={32} className="w-full h-full object-cover" />
                              </div>
                              <div className="w-12 h-8 bg-white dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded overflow-hidden">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/dupont_connectors.avif" alt="Fils Dupont" width={48} height={32} className="w-full h-full object-cover" />
                              </div>
                            </div>
                            <div className="flex-1">
                              <span className="text-sm text-gray-600 dark:text-gray-400">Plaque d'essai sans soudure et câbles de liaison pour assemblage rapide et modifications itératives</span>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    <div className="space-y-8">
                      <h3 id={slugify("2.2 Outils Logiciels")} className="text-2xl font-semibold mb-8 text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-3">
                        2.2 Outils Logiciels
                      </h3>
                      
                      {/* Arduino IDE - Section principale */}
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                        <div className="p-6">
                          <div className="grid lg:grid-cols-3 gap-6">
                            <div className="lg:col-span-2 space-y-4">
                              <div>
                                <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-2">Arduino IDE</h4>
                                <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
                                  Environnement de développement intégré officiel pour la programmation des microcontrôleurs Arduino. 
                                  Interface C/C++ simplifiée avec moniteur série intégré pour le débogage en temps réel.
                                </p>
                              </div>
                              <div className="flex flex-col sm:flex-row sm:items-center gap-2">
                                <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Téléchargement :</span>
                                <a href="https://www.arduino.cc/en/software" target="_blank" rel="noopener noreferrer" 
                                   className="text-sm text-blue-600 dark:text-blue-400 hover:text-blue-700 dark:hover:text-blue-300 underline inline-flex items-center gap-1">
                                  <ExternalLink className="w-3 h-3" />
                                  arduino.cc/en/software
                                </a>
                              </div>
                            </div>
                            <div className="lg:col-span-1">
                              <div className="bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/Arduino-IDE-Interface.png" alt="Interface Arduino IDE" width={300} height={200} className="w-full h-32 object-cover" />
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      {/* Bibliothèques et outils */}
                      <div className="space-y-4">
                        <h4 className="text-lg font-medium text-gray-900 dark:text-gray-100">Bibliothèques et outils de développement</h4>
                        
                        <div className="space-y-3">
                          {/* I2Cdevlib */}
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-start justify-between">
                              <div className="flex-1">
                                <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-1">I2Cdevlib / MPU6050</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                  Bibliothèque spécialisée pour la communication avec le MPU-6050. Inclut les fonctions de calibration, 
                                  d'accès aux registres et l'interface avec le Digital Motion Processor (DMP).
                                </p>
                                <a href="https://github.com/jrowberg/i2cdevlib" target="_blank" rel="noopener noreferrer" 
                                   className="text-xs text-blue-600 dark:text-blue-400 hover:text-blue-700 inline-flex items-center gap-1">
                                  <ExternalLink className="w-3 h-3" />
                                  Dépôt GitHub
                                </a>
                              </div>
                            </div>
                          </div>

                          {/* LiquidCrystal */}
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-start justify-between">
                              <div className="flex-1">
                                <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-1">LiquidCrystal</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                  Bibliothèque native Arduino pour le contrôle des écrans LCD basés sur le contrôleur HD44780. 
                                  Support des interfaces 4 et 8 bits.
                                </p>
                                <span className="inline-flex items-center px-2 py-1 rounded text-xs bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-300">
                                  Inclus dans Arduino IDE
                                </span>
                              </div>
                            </div>
                          </div>

                          {/* KiCad */}
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-start justify-between">
                              <div className="flex-1">
                                <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-1">KiCad EDA</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                  Suite logicielle open-source pour la conception de circuits électroniques : 
                                  schématique, routage PCB, et visualisation 3D.
                                </p>
                                <a href="https://kicad.org/download/" target="_blank" rel="noopener noreferrer" 
                                   className="text-xs text-blue-600 dark:text-blue-400 hover:text-blue-700 inline-flex items-center gap-1">
                                  <ExternalLink className="w-3 h-3" />
                                  kicad.org/download
                                </a>
                              </div>
                            </div>
                          </div>

                          {/* Git/GitHub */}
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-start justify-between">
                              <div className="flex-1">
                                <h5 className="font-medium text-gray-900 dark:text-gray-100 mb-1">Git / GitHub</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                  Système de contrôle de version distribué et plateforme d'hébergement 
                                  pour la gestion collaborative du code source.
                                </p>
                                <div className="flex gap-4">
                                  <a href="https://git-scm.com/" target="_blank" rel="noopener noreferrer" 
                                     className="text-xs text-blue-600 dark:text-blue-400 hover:text-blue-700 inline-flex items-center gap-1">
                                    <ExternalLink className="w-3 h-3" />
                                    Git SCM
                                  </a>
                                  <a href="https://github.com" target="_blank" rel="noopener noreferrer" 
                                     className="text-xs text-blue-600 dark:text-blue-400 hover:text-blue-700 inline-flex items-center gap-1">
                                    <ExternalLink className="w-3 h-3" />
                                    GitHub
                                  </a>
                                </div>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>
                    
                    <div className="space-y-8">
                      <h3 id={slugify("2.3 Schéma de connexion")} className="text-2xl font-semibold mb-8 text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-3">
                        2.3 Schéma de connexion
                      </h3>
                      
                      {/* MPU-6050 */}
                      <div className="space-y-4">
                        <h4 id={slugify("2.3.1 MPU-6050  Arduino")} className="text-lg font-medium text-gray-900 dark:text-gray-100">
                          2.3.1 MPU-6050 ↔ Arduino
                        </h4>
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                          <table className="w-full">
                            <thead className="bg-gray-50 dark:bg-gray-800">
                              <tr>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Broche MPU-6050</th>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Broche Arduino</th>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Fonction</th>
                              </tr>
                            </thead>
                            <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">VDD</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">5V</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Alimentation +5 V</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">GND</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">GND</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Masse</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">SDA</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">A4 (SDA)</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Données I²C</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">SCL</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">A5 (SCL)</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Horloge I²C</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">(AD0)</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">GND</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Adresse I²C = 0x68</td></tr>
                            </tbody>
                          </table>
                        </div>
                      </div>

                      {/* LCD */}
                      <div className="space-y-4">
                        <h4 id={slugify("2.3.2 Ecran LCD 16x2  Arduino")} className="text-lg font-medium text-gray-900 dark:text-gray-100">
                          2.3.2 Écran LCD 16×2 ↔ Arduino
                        </h4>
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                          <table className="w-full">
                            <thead className="bg-gray-50 dark:bg-gray-800">
                              <tr>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Broche LCD</th>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Fonction</th>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Broche Arduino</th>
                              </tr>
                            </thead>
                            <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">VSS</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Masse</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">GND</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">VDD</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Alimentation +5V</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">5V</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">V0</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Contraste</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">Curseur de pot.</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">RS</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Register Select</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">D12</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">E</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Enable</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">D11</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">D4</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Data bit 4</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">D5</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">D5</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Data bit 5</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">D4</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">D6</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Data bit 6</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">D3</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">D7</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Data bit 7</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">D2</td></tr>
                              <tr><td className="px-4 py-3 text-sm font-mono text-gray-900 dark:text-gray-100">RW</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Read/Write (écriture)</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">GND</td></tr>
                            </tbody>
                          </table>
                        </div>
                      </div>

                      {/* Alimentation */}
                      <div className="space-y-4">
                        <h4 id={slugify("2.3.3 Alimentation")} className="text-lg font-medium text-gray-900 dark:text-gray-100">
                          2.3.3 Alimentation
                        </h4>
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg overflow-hidden">
                          <table className="w-full">
                            <thead className="bg-gray-50 dark:bg-gray-800">
                              <tr>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Élément</th>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Connexion</th>
                                <th className="px-4 py-3 text-left text-sm font-medium text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700">Remarque</th>
                              </tr>
                            </thead>
                            <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                              <tr><td className="px-4 py-3 text-sm text-gray-900 dark:text-gray-100">Pile 9V</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Entrée de l'Arduino (barrel jack)</td><td className="px-4 py-3 text-sm font-mono text-gray-700 dark:text-gray-300">9V</td></tr>
                              <tr><td className="px-4 py-3 text-sm text-gray-900 dark:text-gray-100">Masse commune</td><td className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">Masse → rail GND breadboard</td><td className="px-4 py-3 text-sm text-gray-700 dark:text-gray-300">Référence 0V</td></tr>
                            </tbody>
                          </table>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 3 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle id="3-principe-de-fonctionnement-du-mpu-6050" className="scroll-mt-24 text-xl text-gray-900 dark:text-gray-100">
                      3. Principe de fonctionnement du MPU-6050
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Fonctionnement technique de l'accéléromètre et du gyroscope
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Le MPU-6050 est un système intégré combinant un accéléromètre 3 axes et un gyroscope 3 axes, 
                        contrôlé par un microprocesseur interne appelé Digital Motion Processor (DMP). Cette architecture 
                        permet une acquisition et un traitement des données de mouvement en temps réel.
                      </p>
                    </div>

                    {/* 3.1 Accéléromètre */}
                    <div className="space-y-4">
                      <h3 id={slugify("3.1 Accéléromètre")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        3.1 Accéléromètre
                      </h3>
                      
                      <div className="space-y-4">
                        <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                          L'accéléromètre intégré mesure les forces d'accélération selon trois axes orthogonaux :
                        </p>
                        
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <div className="grid md:grid-cols-3 gap-4 text-sm">
                            <div className="text-center">
                              <div className="font-medium text-gray-900 dark:text-gray-100">Axe X</div>
                              <div className="text-gray-600 dark:text-gray-400">Avant-arrière</div>
                            </div>
                            <div className="text-center">
                              <div className="font-medium text-gray-900 dark:text-gray-100">Axe Y</div>
                              <div className="text-gray-600 dark:text-gray-400">Gauche-droite</div>
                            </div>
                            <div className="text-center">
                              <div className="font-medium text-gray-900 dark:text-gray-100">Axe Z</div>
                              <div className="text-gray-600 dark:text-gray-400">Haut-bas</div>
                            </div>
                          </div>
                        </div>

                        <div className="space-y-3">
                          <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                            <strong>Principe de fonctionnement :</strong> Au repos sur une surface horizontale, 
                            l'axe vertical (Z) détecte la force gravitationnelle terrestre (+1 g ≈ 9,81 m/s²). 
                            L'inclinaison du capteur modifie la répartition de cette force sur les axes X et Y, 
                            permettant le calcul de l'orientation spatiale.
                          </p>

                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Configuration de sensibilité</h4>
                            <div className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                              <div>• <strong>±2 g :</strong> Résolution maximale (16 384 LSB/g) - Configuration utilisée</div>
                              <div>• <strong>±4 g, ±8 g, ±16 g :</strong> Plages étendues avec résolution réduite</div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 3.2 Gyroscope */}
                    <div className="space-y-4">
                      <h3 id={slugify("3.2 Gyroscope")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        3.2 Gyroscope
                      </h3>
                      
                      <div className="space-y-4">
                        <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                          Le gyroscope MEMS intégré mesure les vitesses de rotation angulaire selon trois axes :
                        </p>
                        
                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <div className="grid md:grid-cols-3 gap-4 text-sm">
                            <div className="text-center">
                              <div className="font-medium text-gray-900 dark:text-gray-100">Rotation X</div>
                              <div className="text-gray-600 dark:text-gray-400">Roulis (Roll)</div>
                            </div>
                            <div className="text-center">
                              <div className="font-medium text-gray-900 dark:text-gray-100">Rotation Y</div>
                              <div className="text-gray-600 dark:text-gray-400">Tangage (Pitch)</div>
                            </div>
                            <div className="text-center">
                              <div className="font-medium text-gray-900 dark:text-gray-100">Rotation Z</div>
                              <div className="text-gray-600 dark:text-gray-400">Lacet (Yaw)</div>
                            </div>
                          </div>
                        </div>

                        <div className="space-y-3">
                          <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                            <strong>Principe physique :</strong> Une structure vibrante interne subit la force de Coriolis 
                            lors des rotations, générant une tension électrique proportionnelle à la vitesse angulaire.
                          </p>

                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Plages de mesure</h4>
                            <div className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                              <div>• <strong>±250 °/s :</strong> Précision maximale (131 LSB/°/s) - Configuration utilisée</div>
                              <div>• <strong>±500, ±1000, ±2000 °/s :</strong> Plages étendues pour rotations rapides</div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 3.3 Communication I2C */}
                    <div className="space-y-4">
                      <h3 id={slugify("3.3 Registre et communication I2C")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        3.3 Registres et communication I²C
                      </h3>
                      
                      <div className="space-y-4">
                        <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                          Le MPU-6050 utilise le protocole I²C pour la communication avec le microcontrôleur hôte. 
                          Ce bus série à deux fils (SDA/SCL) avec résistances de pull-up permet un échange de données 
                          fiable à l'adresse standard <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">0x68</code>.
                        </p>

                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Séquence de lecture des données</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div className="flex items-start gap-2">
                              <span className="text-gray-400 mt-1">1.</span>
                              <span>Transmission START + adresse capteur (0x68)</span>
                            </div>
                            <div className="flex items-start gap-2">
                              <span className="text-gray-400 mt-1">2.</span>
                              <span>Spécification du registre de départ (ex: <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">ACCEL_XOUT_H</code>)</span>
                            </div>
                            <div className="flex items-start gap-2">
                              <span className="text-gray-400 mt-1">3.</span>
                              <span>Lecture séquentielle des 12 octets de données (6 valeurs × 2 octets)</span>
                            </div>
                          </div>
                        </div>

                        <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Structure des données</h4>
                          <div className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                            <div>• Accélération X, Y, Z (16 bits signés chacune)</div>
                            <div>• Vitesse angulaire X, Y, Z (16 bits signés chacune)</div>
                            <div>• Plage de valeurs : -32 768 à +32 767 LSB</div>
                            <div>• Interface simplifiée via <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">getMotion6()</code></div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 3.4 Conversion */}
                    <div className="space-y-4">
                      <h3 id={slugify("3.4 Conversion des données brutes")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        3.4 Conversion des données brutes
                      </h3>
                      
                      <div className="space-y-4">
                        <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                          Les valeurs numériques brutes (LSB) doivent être converties en unités physiques exploitables :
                        </p>

                        <div className="grid md:grid-cols-2 gap-4">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Accélération</h4>
                            <div className="space-y-2 text-sm font-mono text-gray-600 dark:text-gray-400">
                              <div>a_g = LSB / 16384</div>
                              <div>a_m/s² = a_g × 9.81</div>
                            </div>
                          </div>
                          
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Vitesse angulaire</h4>
                            <div className="space-y-2 text-sm font-mono text-gray-600 dark:text-gray-400">
                              <div>ω_°/s = LSB / 131</div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 3.5 Filtrage */}
                    <div className="space-y-4">
                      <h3 id={slugify("3.5 Filtrage et fusion")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        3.5 Filtrage et fusion de données
                      </h3>
                      
                      <div className="space-y-4">
                        <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                          Le traitement des données brutes nécessite l'application de filtres pour éliminer le bruit 
                          et combiner les informations des deux capteurs.
                        </p>

                        <div className="space-y-4">
                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Filtre passe-bas numérique (DLPF)</h4>
                            <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                              Atténuation des hautes fréquences pour réduire le bruit de mesure.
                            </p>
                            <div className="text-sm text-gray-600 dark:text-gray-400">
                              Configuration : 20 Hz ou 42 Hz selon les besoins de réactivité
                            </div>
                          </div>

                          <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Filtre complémentaire</h4>
                            <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                              Fusion des données accéléromètre (stabilité long terme) et gyroscope (réactivité court terme).
                            </p>
                            <div className="bg-gray-50 dark:bg-gray-900/30 rounded p-3">
                              <div className="text-sm font-mono text-gray-700 dark:text-gray-300">
                                θ = α(θ + ωΔt) + (1-α)θ_acc
                              </div>
                              <div className="text-xs text-gray-500 dark:text-gray-400 mt-1">
                                avec α ≈ 0,94 (coefficient de fusion)
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 4 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle id="4-architecture-du-système" className="scroll-mt-24 text-xl text-gray-900 dark:text-gray-100">
                      4. Architecture du système
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Organisation fonctionnelle et flux de données du système embarqué
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        L'architecture système définit l'interaction entre les composants matériels et logiciels 
                        pour transformer les données de mouvement brutes en informations exploitables et affichage en temps réel.
                      </p>
                    </div>

                    {/* 4.1 Alimentation */}
                    <div className="space-y-4">
                      <h3 id={slugify("4.1 Alimentation")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        4.1 Sous-système d'alimentation
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <div className="space-y-3">
                          <div className="flex items-center gap-3">
                            <div className="w-3 h-3 bg-gray-400 rounded-full"></div>
                            <span className="text-gray-900 dark:text-gray-100 font-medium">Source primaire :</span>
                            <span className="text-gray-600 dark:text-gray-400">Pile rechargeable 9V</span>
                          </div>
                          
                          <div className="ml-6 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Distribution :</strong></div>
                            <div className="ml-4 space-y-1">
                              <div>• Arduino Uno (via barrel jack)</div>
                              <div>• MPU-6050 (via rail +5V Arduino)</div>
                              <div>• Écran LCD 16×2 (via rail +5V Arduino)</div>
                            </div>
                            <div><strong>Référence commune :</strong> Rail GND partagé pour stabilité des mesures</div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 4.2 Microcontrôleur */}
                    <div className="space-y-4">
                      <h3 id={slugify("4.2 Microcontrôleur Arduino")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        4.2 Unité de traitement (Arduino Uno)
                      </h3>
                      
                      <div className="grid md:grid-cols-3 gap-4">
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Maître I²C</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Génère les séquences de lecture du MPU-6050 et récupère les données brutes.
                          </p>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Processeur de données</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Conversion, filtrage, fusion des capteurs et détection de mouvements.
                          </p>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Interface utilisateur</h4>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Pilotage LCD temps réel et communication série pour débogage.
                          </p>
                        </div>
                      </div>
                    </div>

                    {/* 4.3 Capteur */}
                    <div className="space-y-4">
                      <h3 id={slugify("4.3 Capteur MPU-6050")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        4.3 Module de capture (MPU-6050)
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <div className="space-y-3">
                          <div className="grid md:grid-cols-2 gap-4 text-sm">
                            <div>
                              <span className="font-medium text-gray-900 dark:text-gray-100">Acquisition continue :</span>
                              <span className="text-gray-600 dark:text-gray-400 ml-2">Fréquence configurée (~100 Hz)</span>
                            </div>
                            <div>
                              <span className="font-medium text-gray-900 dark:text-gray-100">Stockage interne :</span>
                              <span className="text-gray-600 dark:text-gray-400 ml-2">Registres I²C accessibles</span>
                            </div>
                            <div>
                              <span className="font-medium text-gray-900 dark:text-gray-100">Prétraitement :</span>
                              <span className="text-gray-600 dark:text-gray-400 ml-2">DLPF matériel intégré</span>
                            </div>
                            <div>
                              <span className="font-medium text-gray-900 dark:text-gray-100">Extension DMP :</span>
                              <span className="text-gray-600 dark:text-gray-400 ml-2">Fusion avancée (optionnelle)</span>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 4.4 Interfaces */}
                    <div className="space-y-4">
                      <h3 id={slugify("4.4 Affichage et debug")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        4.4 Interfaces de sortie
                      </h3>
                      
                      <div className="grid md:grid-cols-2 gap-4">
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Écran LCD 16×2</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Ligne 1 :</strong> Type de mouvement détecté</div>
                            <div className="ml-4 text-xs">MONTEE, VIRAGE, VOL STABLE, etc.</div>
                            <div><strong>Ligne 2 :</strong> Données quantitatives</div>
                            <div className="ml-4 text-xs">Score Jerk + Accélération totale (m/s²)</div>
                          </div>
                        </div>
                        
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Moniteur série (USB)</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Débogage temps réel :</strong></div>
                            <div className="ml-4 text-xs">• Angles calculés (roulis, tangage)</div>
                            <div className="ml-4 text-xs">• Vitesses angulaires brutes</div>
                            <div className="ml-4 text-xs">• Valeurs d'accélération filtrées</div>
                            <div className="ml-4 text-xs">• États de calibration</div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 4.5 Flux de données */}
                    <div className="space-y-4">
                      <h3 id={slugify("4.5 Flux de données")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        4.5 Pipeline de traitement des données
                      </h3>
                      
                      <div className="space-y-3">
                        {[
                          {
                            stage: "Acquisition",
                            description: "Lecture des registres via getMotion6()",
                            details: "ax, ay, az, gx, gy, gz (valeurs brutes 16-bit)"
                          },
                          {
                            stage: "Calibration",
                            description: "Correction des biais et références",
                            details: "Soustraction offsets gyroscope, référence accéléromètre"
                          },
                          {
                            stage: "Conversion",
                            description: "Transformation en unités physiques",
                            details: "LSB → g → m/s², LSB → °/s"
                          },
                          {
                            stage: "Filtrage",
                            description: "Réduction du bruit et fusion",
                            details: "DLPF matériel + filtre complémentaire logiciel"
                          },
                          {
                            stage: "Classification",
                            description: "Détection de mouvements",
                            details: "Seuils de décision + calcul du Jerk"
                          },
                          {
                            stage: "Affichage",
                            description: "Interfaces utilisateur",
                            details: "LCD temps réel + moniteur série debug"
                          }
                        ].map((step, index) => (
                          <div key={index} className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-start gap-4">
                              <div className="w-8 h-8 bg-gray-100 dark:bg-gray-800 rounded-full flex items-center justify-center flex-shrink-0">
                                <span className="text-sm font-medium text-gray-600 dark:text-gray-400">{index + 1}</span>
                              </div>
                              <div className="flex-1">
                                <div className="flex items-center gap-2 mb-1">
                                  <h4 className="font-medium text-gray-900 dark:text-gray-100">{step.stage}</h4>
                                  <span className="text-sm text-gray-500 dark:text-gray-400">→</span>
                                  <span className="text-sm text-gray-600 dark:text-gray-400">{step.description}</span>
                                </div>
                                <p className="text-xs text-gray-500 dark:text-gray-400">{step.details}</p>
                              </div>
                            </div>
                          </div>
                        ))}
                      </div>

                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4 mt-4">
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          <strong>Exécution cyclique :</strong> L'ensemble de ce pipeline s'exécute dans la boucle principale 
                          (<code className="bg-gray-100 dark:bg-gray-800 px-1 rounded">loop()</code>) avec une période 
                          d'environ 100ms, assurant un système réactif et une mise à jour fluide de l'affichage.
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 5 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle id="5-schéma-électrique-kicad" className="scroll-mt-24 text-xl text-gray-900 dark:text-gray-100">
                      5. Schéma Électrique (KiCad)
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Conception et documentation du schéma électronique
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Pour garantir la clarté, la reproductibilité et la documentation technique du montage, 
                        un schéma électronique complet a été réalisé avec KiCad 7.0.
                      </p>
                    </div>

                    {/* Schéma principal */}
                    <div className="space-y-4">
                      <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        Schéma électronique principal
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-1/electronique/images/Kicade-Schema-Official.png" alt="Schéma électronique KiCad" width={800} height={600} className="rounded-md mx-auto w-full object-contain" />
                      </div>
                    </div>

                    {/* Description des blocs fonctionnels */}
                    <div className="space-y-6">
                      <h3 id={slugify("5.1 Description des parties du schéma")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        5.1 Description des blocs fonctionnels
                      </h3>
                      
                      <div className="grid gap-4">
                        {/* Bloc alimentation */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Bloc alimentation</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Entrée :</strong> Pile 9V → connecteur barrel jack Arduino</div>
                            <div><strong>Régulation :</strong> Régulateur interne Arduino (7805) → +5V stabilisé</div>
                            <div><strong>Distribution :</strong> Rails d'alimentation breadboard (+5V, GND)</div>
                          </div>
                        </div>

                        {/* Arduino Uno */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Microcontrôleur Arduino Uno</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Communication I²C :</strong> A4 (SDA), A5 (SCL) → MPU-6050</div>
                            <div><strong>Interface LCD :</strong> D2, D3, D4, D5, D11, D12 → HD44780</div>
                            <div><strong>Alimentation :</strong> +5V, GND distribués aux périphériques</div>
                          </div>
                        </div>

                        {/* MPU-6050 */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Module capteur MPU-6050</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Alimentation :</strong> VCC → +5V, GND → GND</div>
                            <div><strong>Adressage :</strong> AD0 → GND (adresse I²C = 0x68)</div>
                            <div><strong>Interruption :</strong> INT non connectée (polling mode)</div>
                          </div>
                        </div>

                        {/* LCD */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Afficheur LCD 16×2 (HD44780)</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Alimentation :</strong> VSS → GND, VDD → +5V</div>
                            <div><strong>Contraste :</strong> V0 → curseur potentiomètre 10kΩ</div>
                            <div><strong>Interface 4-bit :</strong> RS → D12, E → D11, D4-D7 → D5,D4,D3,D2</div>
                            <div><strong>Mode écriture :</strong> RW → GND (lecture seule non utilisée)</div>
                          </div>
                        </div>

                        {/* Potentiomètre */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Potentiomètre de contraste (10kΩ)</h4>
                          <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                            <div><strong>Configuration :</strong> Diviseur de tension variable</div>
                            <div><strong>Connexions :</strong> Borne gauche → +5V, borne droite → GND</div>
                            <div><strong>Sortie :</strong> Curseur central → broche V0 du LCD</div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* Fichiers projet */}
                    <div className="space-y-4">
                      <h3 id={slugify("5.2 Fichiers KiCad")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        5.2 Fichiers de conception
                      </h3>
                      
                      <div className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                        <div className="space-y-3 text-sm">
                          <div className="flex items-center gap-3">
                            <span className="font-medium text-gray-900 dark:text-gray-100">Projet KiCad :</span>
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-gray-700 dark:text-gray-300">Kicad/</code>
                          </div>
                          <div className="flex items-center gap-3">
                            <span className="font-medium text-gray-900 dark:text-gray-100">Schématique :</span>
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-gray-700 dark:text-gray-300">Test1_MPU6050_Schema.kicad_sch</code>
                          </div>
                          <div className="flex items-center gap-3">
                            <span className="font-medium text-gray-900 dark:text-gray-100">Documentation :</span>
                            <span className="text-gray-600 dark:text-gray-400">Netlist exportée, BOM générée</span>
                          </div>
                        </div>
                        
                        <div className="mt-4 p-3 bg-blue-50 dark:bg-blue-950/20 border border-blue-200 dark:border-blue-800 rounded">
                          <p className="text-sm text-gray-700 dark:text-gray-300">
                            <strong>Accès aux fichiers :</strong> Le projet complet est disponible dans le dépôt GitHub. 
                            Les symboles utilisent les bibliothèques officielles KiCad avec annotations adaptées 
                            pour la clarification des noms de broches.
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 6 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle id="6-installation-et-compilation-du-code" className="scroll-mt-24 text-xl text-gray-900 dark:text-gray-100">
                      6. Installation et compilation du code
                    </CardTitle>
                    <CardDescription className="text-gray-600 dark:text-gray-400">
                      Configuration de l'environnement de développement et déploiement
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Cette section détaille les étapes d'installation des outils de développement, 
                        la configuration des bibliothèques requises et la procédure de compilation du firmware.
                      </p>
                    </div>

                    {/* Prérequis logiciels */}
                    <div className="space-y-6">
                      <h3 id={slugify("6.1 Prérequis logiciels")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        6.1 Environnement de développement
                      </h3>
                      
                      <div className="space-y-4">
                        {/* Arduino IDE */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Arduino IDE (version ≥ 1.8.19)</h4>
                          <div className="space-y-2">
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Environnement de développement intégré pour la programmation des microcontrôleurs Arduino.
                            </p>
                            <div className="flex items-center gap-2">
                              <span className="text-sm font-medium text-gray-700 dark:text-gray-300">Téléchargement :</span>
                              <a href="https://www.arduino.cc/en/software" target="_blank" rel="noopener noreferrer" 
                                 className="text-sm text-blue-600 dark:text-blue-400 hover:text-blue-700 inline-flex items-center gap-1">
                                <ExternalLink className="w-3 h-3" />
                                arduino.cc/en/software
                              </a>
                            </div>
                          </div>
                        </div>

                        {/* Bibliothèques */}
                        <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                          <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-3">Bibliothèques requises</h4>
                          
                          <div className="space-y-4">
                            <div className="border-l-2 border-gray-200 dark:border-gray-700 pl-4">
                              <h5 className="font-medium text-gray-900 dark:text-gray-100 text-sm">I2Cdevlib</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Bibliothèque pour I2Cdev.h et MPU6050.h - Interface de communication avec le capteur
                              </p>
                              <div className="bg-gray-900 dark:bg-gray-950 rounded p-3 text-sm font-mono text-gray-100 overflow-x-auto">
                                <div>cd ~/Arduino/libraries</div>
                                <div>git clone https://github.com/jrowberg/i2cdevlib.git</div>
                              </div>
                            </div>
                            
                            <div className="border-l-2 border-gray-200 dark:border-gray-700 pl-4">
                              <h5 className="font-medium text-gray-900 dark:text-gray-100 text-sm">LiquidCrystal</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Bibliothèque standard Arduino pour contrôleur HD44780 - 
                                <span className="inline-flex items-center ml-2 px-2 py-1 rounded text-xs bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-300">
                                  Incluse dans Arduino IDE
                                </span>
                              </p>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* Procédure de compilation */}
                    <div className="space-y-6">
                      <h3 id={slugify("6.2 Importer et compiler")} className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        6.2 Procédure de compilation et déploiement
                      </h3>
                      
                      <div className="space-y-4">
                        {[
                          {
                            step: "1",
                            title: "Lancement de l'IDE",
                            description: "Ouvrir Arduino IDE et vérifier la version"
                          },
                          {
                            step: "2", 
                            title: "Chargement du projet",
                            description: "Ouvrir le fichier Test1_MPU6050.ino dans Arduino/Test1_MPU6050/"
                          },
                          {
                            step: "3",
                            title: "Vérification des bibliothèques", 
                            description: "Menu Croquis → Inclure une bibliothèque : vérifier I2Cdevlib et LiquidCrystal"
                          },
                          {
                            step: "4",
                            title: "Configuration matérielle",
                            description: "Outils → Type de carte → Arduino Uno"
                          },
                          {
                            step: "5",
                            title: "Sélection du port",
                            description: "Port série correspondant (COMx sous Windows, /dev/ttyACM0 sous Linux)"
                          },
                          {
                            step: "6",
                            title: "Compilation et téléversement", 
                            description: "Clic sur Téléverser (→) pour compiler et programmer la carte"
                          }
                        ].map((item, index) => (
                          <div key={index} className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                            <div className="flex items-start gap-4">
                              <div className="w-8 h-8 bg-gray-100 dark:bg-gray-800 rounded-full flex items-center justify-center flex-shrink-0">
                                <span className="text-sm font-medium text-gray-600 dark:text-gray-400">{item.step}</span>
                              </div>
                              <div className="flex-1">
                                <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-1">{item.title}</h4>
                                <p className="text-sm text-gray-600 dark:text-gray-400">{item.description}</p>
                              </div>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>

                    {/* Validation */}
                    <div className="bg-green-50 dark:bg-green-950/20 border border-green-200 dark:border-green-800 rounded-lg p-4">
                      <h4 className="font-medium text-gray-900 dark:text-gray-100 mb-2">Validation du déploiement</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        Après un téléversement réussi, l'écran LCD doit afficher 
                        <code className="bg-gray-100 dark:bg-gray-800 px-1 rounded mx-1">"MAQUETTE AVION"</code> 
                        pendant 3 secondes avant de passer aux mesures en temps réel.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 7 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle id="7-explication-détaillée-du-code" className="flex items-center gap-2 scroll-mt-24">
                        <Code className="w-5 h-5 text-gray-600" />
                        7. Explication détaillée du code
                    </CardTitle>
                    <CardDescription>
                      Analyse approfondie de l'architecture logicielle et des algorithmes implémentés
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8">
                    
                    {/* Introduction */}
                    <div className="bg-gray-50 dark:bg-gray-900 p-6 rounded-lg border-l-4 border-gray-400">
                      <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                        Cette section présente une analyse détaillée du code source, organisée selon l'ordre d'exécution du programme. 
                        Chaque module et fonction est expliqué avec ses objectifs, paramètres et interactions avec le système global.
                      </p>
                    </div>

                    {/* 7.1 En-tête et bibliothèques */}
                    <div className="space-y-4">
                      <h3 id={slugify("7.1 En-tête et bibliothèques")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        7.1 En-tête et bibliothèques
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Dépendances logicielles</h4>
                        <div className="grid gap-4">
                          <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                            <div className="flex items-center gap-2 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded text-sm">&lt;Wire.h&gt;</code>
                              <span className="text-sm text-gray-600 dark:text-gray-400">Communication I²C</span>
                            </div>
                            <p className="text-gray-700 dark:text-gray-300 text-sm">Gestion des communications I²C entre l'Arduino et le MPU-6050</p>
                          </div>
                          
                          <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                            <div className="flex items-center gap-2 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded text-sm">&lt;I2Cdev.h&gt;</code>
                              <span className="text-sm text-gray-600 dark:text-gray-400">Interface générique I²C</span>
                            </div>
                            <p className="text-gray-700 dark:text-gray-300 text-sm">Couche d'abstraction pour les périphériques I²C</p>
                          </div>
                          
                          <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                            <div className="flex items-center gap-2 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded text-sm">&lt;MPU6050.h&gt;</code>
                              <span className="text-sm text-gray-600 dark:text-gray-400">Pilote capteur</span>
                            </div>
                            <p className="text-gray-700 dark:text-gray-300 text-sm">Fonctions spécialisées pour le contrôle du MPU-6050</p>
                          </div>
                          
                          <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                            <div className="flex items-center gap-2 mb-2">
                              <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded text-sm">&lt;LiquidCrystal.h&gt;</code>
                              <span className="text-sm text-gray-600 dark:text-gray-400">Affichage LCD</span>
                            </div>
                            <p className="text-gray-700 dark:text-gray-300 text-sm">Interface avec l'écran LCD HD44780 16×2</p>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 7.2 Variables globales */}
                    <div className="space-y-4">
                      <h3 id={slugify("7.2 Déclaration des objets et variables globales")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        7.2 Architecture des données globales
                      </h3>
                      
                      <div className="grid gap-6">
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Instances d'objets</h4>
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded border-l-4 border-blue-500">
                              <code className="text-blue-700 dark:text-blue-300">MPU6050 capteurMouvement;</code>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">Instance principale du capteur inertiel</p>
                            </div>
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded border-l-4 border-green-500">
                              <code className="text-green-700 dark:text-green-300">LiquidCrystal ecran(...);</code>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">Interface d'affichage LCD 16×2</p>
                            </div>
                          </div>
                        </div>

                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Variables de calibration</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Correction gyroscope</h5>
                              <ul className="space-y-2 text-sm">
                                <li><code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">erreurGyroX/Y/Z</code> - Biais de dérive</li>
                              </ul>
                            </div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Référence accéléromètre</h5>
                              <ul className="space-y-2 text-sm">
                                <li><code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">referenceAccelX/Y</code> - Position repos</li>
                              </ul>
                            </div>
                          </div>
                        </div>

                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Constantes de conversion</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <div className="flex items-center justify-between mb-2">
                                <code className="text-gray-800 dark:text-gray-200">CONVERSION_ACCEL</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">16384.0</span>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400">Facteur LSB → g (±2g range)</p>
                            </div>
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <div className="flex items-center justify-between mb-2">
                                <code className="text-gray-800 dark:text-gray-200">CONVERSION_GYRO</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">131.0</span>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400">Facteur LSB → °/s (±250°/s range)</p>
                            </div>
                          </div>
                        </div>

                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Paramètres de fusion</h4>
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <div className="flex items-center justify-between mb-2">
                                <code className="text-gray-800 dark:text-gray-200">FORCE_FILTRE</code>
                                <span className="text-sm text-gray-600 dark:text-gray-400">0.94</span>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400">Coefficient filtre complémentaire (94% gyro, 6% accéléro)</p>
                            </div>
                            <div className="grid md:grid-cols-2 gap-4 mt-3">
                              <div>
                                <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded text-sm">angleRoulis</code>
                                <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Angle de roulis fusionné</p>
                              </div>
                              <div>
                                <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded text-sm">angleTangage</code>
                                <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Angle de tangage fusionné</p>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 7.3 Calibration */}
                    <div className="space-y-4">
                      <h3 id={slugify("7.3 Calibration calibrerCapteur")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        7.3 Procédure de calibration
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Fonction <code>calibrerCapteur()</code></h4>
                        
                        <div className="space-y-4">
                          <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg">
                            <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-3">Algorithme de calibration</h5>
                            <ol className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                              <li><strong>1.</strong> Accumulation de 1000 échantillons sur tous les axes</li>
                              <li><strong>2.</strong> Affichage de progression (points dans le moniteur série)</li>
                              <li><strong>3.</strong> Calcul des moyennes arithmétiques</li>
                              <li><strong>4.</strong> Stockage des valeurs de correction</li>
                              <li><strong>5.</strong> Initialisation des filtres moyennes mobiles</li>
                            </ol>
                          </div>
                          
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h6 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Correction gyroscope</h6>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Les valeurs <code>erreurGyroX/Y/Z</code> compensent la dérive statique
                              </p>
                            </div>
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h6 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Référence accéléromètre</h6>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Les valeurs <code>referenceAccelX/Y</code> définissent la position de repos
                              </p>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 7.4 Setup */}
                    <div className="space-y-4">
                      <h3 id={slugify("7.4 setup")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        7.4 Initialisation système - <code>setup()</code>
                      </h3>
                      
                      <div className="space-y-4">
                        {/* Séquence d'initialisation */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Séquence d'initialisation</h4>
                          
                          <div className="space-y-3">
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">1</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Communication série</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400">
                                  <code>Serial.begin(9600)</code> - Démarrage du canal de débogage
                                </p>
                              </div>
                            </div>
                            
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">2</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Bus I²C et capteur</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400">
                                  <code>Wire.begin()</code> + <code>initialize()</code> - Activation des communications
                                </p>
                              </div>
                            </div>
                            
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">3</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Test de connectivité</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400">
                                  <code>testConnection()</code> - Vérification du registre WHO_AM_I
                                </p>
                                <div className="bg-red-50 dark:bg-red-900 p-2 mt-2 rounded text-xs">
                                  <strong>Échec:</strong> Message d'erreur sur LCD et blocage du programme
                                </div>
                              </div>
                            </div>
                            
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">4</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Configuration capteur</h5>
                                <div className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                                  <p>• Plage accéléromètre: ±2g</p>
                                  <p>• Plage gyroscope: ±250°/s</p>
                                  <p>• DLPF: 20 Hz (réduction bruit)</p>
                                </div>
                              </div>
                            </div>
                            
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">5</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Calibration automatique</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400">
                                  <code>calibrerCapteur(1000)</code> - Correction des biais
                                </p>
                              </div>
                            </div>
                            
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">6</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Message d'accueil</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400">
                                  Affichage "MAQUETTE AVION / PRET AU VOL" (3 secondes)
                                </p>
                              </div>
                            </div>
                            
                            <div className="flex items-start gap-3">
                              <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">7</div>
                              <div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Initialisation temporelle</h5>
                                <p className="text-sm text-gray-600 dark:text-gray-400">
                                  <code>dernierTemps = millis()</code> - Référence chronomètre
                                </p>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 7.5 Boucle principale */}
                    <div className="space-y-4">
                      <h3 id={slugify("7.5 Boucle principale loop")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        7.5 Boucle principale - <code>loop()</code>
                      </h3>
                      
                      <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg border-l-4 border-gray-400 mb-6">
                        <p className="text-gray-700 dark:text-gray-300 text-sm">
                          <strong>Fréquence:</strong> 10 Hz (cycle de 100 ms) - Optimisée pour l'affichage LCD et la réactivité
                        </p>
                      </div>

                      <div className="space-y-6">
                        {/* Étape 1: Lecture */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">1. Acquisition des données brutes</h4>
                          <div className="bg-gray-900 rounded-lg p-4 overflow-x-auto">
                            <pre className="text-gray-100 text-sm"><code>capteurMouvement.getMotion6(&amp;ax,&amp;ay,&amp;az,&amp;gx,&amp;gy,&amp;gz);</code></pre>
                          </div>
                          <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">
                            Lecture simultanée des 6 axes (3 accéléromètre + 3 gyroscope) en valeurs LSB
                          </p>
                        </div>

                        {/* Étape 2: Temps */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">2. Calcul du temps écoulé</h4>
                          <div className="bg-gray-900 rounded-lg p-4 overflow-x-auto">
                            <pre className="text-gray-100 text-sm"><code>float deltaTemps = (millis() - dernierTemps) / 1000.0;{"\n"}dernierTemps = millis();</code></pre>
                          </div>
                          <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">
                            Calcul de Δt en secondes pour l'intégration gyroscopique
                          </p>
                        </div>

                        {/* Étape 3: Conversion */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">3. Correction et conversion des unités</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Gyroscope</h5>
                              <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded text-sm">
                                <code>(gx - erreurGyroX) / CONVERSION_GYRO</code>
                                <p className="text-gray-600 dark:text-gray-400 mt-1">LSB → °/s après correction biais</p>
                              </div>
                            </div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Accéléromètre</h5>
                              <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded text-sm">
                                <code>ax / CONVERSION_ACCEL</code>
                                <p className="text-gray-600 dark:text-gray-400 mt-1">LSB → g (unité gravitationnelle)</p>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Étape 4: Moyennes mobiles */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">4. Filtrage par moyennes mobiles</h4>
                          <div className="bg-gray-900 rounded-lg p-4 overflow-x-auto mb-3">
                            <pre className="text-gray-100 text-sm"><code>moyenneAccelX = 0.95*moyenneAccelX + 0.05*aX;{"\n"}moyenneAccelY = 0.95*moyenneAccelY + 0.05*aY;</code></pre>
                          </div>
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Filtre passe-bas exponentiel (τ ≈ 2 secondes) pour la détection de glissement
                          </p>
                        </div>

                        {/* Étape 5: Fusion */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">5. Fusion des capteurs (filtre complémentaire)</h4>
                          <div className="space-y-3">
                            <div className="grid md:grid-cols-3 gap-4 text-sm">
                              <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                                <h6 className="font-medium mb-1">Angles accéléromètre</h6>
                                <code>atan2()</code>
                                <p className="text-gray-600 dark:text-gray-400 text-xs mt-1">Référence absolue</p>
                              </div>
                              <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                                <h6 className="font-medium mb-1">Intégration gyroscope</h6>
                                <code>angle += ω·Δt</code>
                                <p className="text-gray-600 dark:text-gray-400 text-xs mt-1">Dynamique rapide</p>
                              </div>
                              <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                                <h6 className="font-medium mb-1">Fusion finale</h6>
                                <code>α·θ_gyro + (1-α)·θ_accel</code>
                                <p className="text-gray-600 dark:text-gray-400 text-xs mt-1">α = 0.94</p>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Étape 6: Détection */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">6. Algorithme de détection de mouvement</h4>
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-900 p-3 rounded border-l-4 border-gray-500">
                              <div className="flex items-center justify-between">
                                <span className="font-medium text-gray-800 dark:text-gray-200">Priorité 1: Inclinaison</span>
                                <code className="text-sm bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">|roll/pitch| &gt; ±12°</code>
                              </div>
                              <p className="text-sm text-gray-700 dark:text-gray-300 mt-1">MONTEE, DESCENTE, VIRAGE GAUCHE/DROITE</p>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-900 p-3 rounded border-l-4 border-gray-500">
                              <div className="flex items-center justify-between">
                                <span className="font-medium text-gray-800 dark:text-gray-200">Priorité 2: Lacet</span>
                                <code className="text-sm bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">|ωz| &gt; 45°/s</code>
                              </div>
                              <p className="text-sm text-gray-700 dark:text-gray-300 mt-1">ROTATION GAUCHE/DROITE</p>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-900 p-3 rounded border-l-4 border-gray-500">
                              <div className="flex items-center justify-between">
                                <span className="font-medium text-gray-800 dark:text-gray-200">Priorité 3: Glissement avant/arrière</span>
                                <code className="text-sm bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">|ΔaX| &gt; 0.15g</code>
                              </div>
                              <p className="text-sm text-gray-700 dark:text-gray-300 mt-1">AVANCE, RECULE</p>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-900 p-3 rounded border-l-4 border-gray-500">
                              <div className="flex items-center justify-between">
                                <span className="font-medium text-gray-800 dark:text-gray-200">Priorité 4: Glissement latéral</span>
                                <code className="text-sm bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">|ΔaY| &gt; 0.15g</code>
                              </div>
                              <p className="text-sm text-gray-700 dark:text-gray-300 mt-1">GLISSE GAUCHE/DROITE</p>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded border-l-4 border-gray-500">
                              <div className="flex items-center justify-between">
                                <span className="font-medium text-gray-800 dark:text-gray-200">Par défaut</span>
                                <code className="text-sm bg-gray-100 dark:bg-gray-600 px-2 py-1 rounded">5 cycles stables</code>
                              </div>
                              <p className="text-sm text-gray-700 dark:text-gray-300 mt-1">VOL STABLE</p>
                            </div>
                          </div>
                        </div>

                        {/* Étape 7: Jerk */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">7. Calcul du Jerk (à-coup)</h4>
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <h6 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Norme de l'accélération</h6>
                              <code className="text-sm">|a| = √(aX² + aY² + aZ²) × 9.81 m/s²</code>
                            </div>
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <h6 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Variation temporelle</h6>
                              <code className="text-sm">Δ|a| = |a|(t) - |a|(t-1)</code>
                            </div>
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <h6 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Score lissé</h6>
                              <code className="text-sm">scoreJerk = 0.8 × scoreJerk + 0.2 × |Δ|a||</code>
                            </div>
                          </div>
                        </div>

                        {/* Étape 8: Affichage */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">8. Interface utilisateur</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Écran LCD</h5>
                              <div className="bg-black text-green-400 p-3 rounded font-mono text-sm">
                                <div>VIRAGE DROITE    </div>
                                <div>J: 3 A: 9.5 m/s²</div>
                              </div>
                            </div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Moniteur série</h5>
                              <div className="bg-gray-100 dark:bg-gray-700 p-3 rounded text-sm">
                                <p>• État du mouvement</p>
                                <p>• Angles (R, T)</p>
                                <p>• Vitesses angulaires</p>
                                <p>• Accélérations et écarts</p>
                                <p>• Score Jerk</p>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 7.6 Fonctions utilitaires */}
                    <div className="space-y-4">
                      <h3 id={slugify("7.6 Fonctions utilitaires")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        7.6 Fonctions utilitaires
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">
                          Fonction <code>remettreAZero()</code>
                        </h4>
                        
                        <div className="space-y-4">
                          <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg">
                            <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-3">Variables réinitialisées</h5>
                            <div className="grid md:grid-cols-2 gap-4 text-sm">
                              <div>
                                <ul className="space-y-1 text-gray-600 dark:text-gray-400">
                                  <li>• <code>angleRoulis = 0</code></li>
                                  <li>• <code>angleTangage = 0</code></li>
                                </ul>
                              </div>
                              <div>
                                <ul className="space-y-1 text-gray-600 dark:text-gray-400">
                                  <li>• <code>scoreJerk = 0</code></li>
                                  <li>• <code>moyenneAccelX/Y</code> (référence)</li>
                                </ul>
                              </div>
                            </div>
                          </div>
                          
                          <div className="bg-yellow-50 dark:bg-yellow-900 p-3 rounded border-l-4 border-yellow-500">
                            <p className="text-yellow-800 dark:text-yellow-200 text-sm">
                              <strong>Affichage:</strong> "REMISE A ZERO" pendant 2 secondes
                            </p>
                          </div>
                        </div>
                      </div>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 8 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle id="8-mode-demploi-et-démonstration" className="flex items-center gap-2 scroll-mt-24">
                        <PlayCircle className="w-5 h-5 text-gray-600" />
                        8. Mode d'emploi et démonstration
                    </CardTitle>
                    <CardDescription>
                      Procédures opérationnelles et tests de validation du système
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8">

                    {/* 8.1 Mise sous tension */}
                    <div className="space-y-4">
                      <h3 id={slugify("8.1 Mise sous tension")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        8.1 Procédure de mise sous tension
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Séquence d'alimentation</h4>
                        <div className="space-y-3">
                          <div className="flex items-start gap-3">
                            <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">1</div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300">Alimentation principale</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Connecter la pile 9V sur l'entrée d'alimentation de l'Arduino Uno
                              </p>
                            </div>
                          </div>
                          
                          <div className="flex items-start gap-3">
                            <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">2</div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300">Liaison de débogage (optionnel)</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Connecter le câble USB pour accéder au moniteur série
                              </p>
                            </div>
                          </div>
                          
                          <div className="flex items-start gap-3">
                            <div className="flex-shrink-0 w-8 h-8 bg-gray-600 text-white rounded-full flex items-center justify-center text-sm font-medium">3</div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300">Activation du système</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Activer l'interrupteur d'alimentation (si présent sur le montage)
                              </p>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 8.2 Séquence opérationnelle */}
                    <div className="space-y-4">
                      <h3 id={slugify("8.2 Déroulé de la séquence")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        8.2 Séquence opérationnelle
                      </h3>
                      
                      <div className="space-y-4">
                        {/* Phase d'initialisation */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Phase d'initialisation</h4>
                          <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg border-l-4 border-blue-500">
                            <div className="flex items-center justify-between mb-2">
                              <span className="font-medium text-blue-800 dark:text-blue-200">Message d'accueil</span>
                              <span className="text-sm text-blue-600 dark:text-blue-400">3 secondes</span>
                            </div>
                            <div className="bg-black text-green-400 p-3 rounded font-mono text-sm">
                              <div>MAQUETTE AVION   </div>
                              <div>PRET AU VOL     </div>
                            </div>
                          </div>
                        </div>

                        {/* Phase de mesure */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Mode de mesure continue</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="bg-gray-50 dark:bg-gray-700 p-4 rounded">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Ligne 1 - État du mouvement</h5>
                              <ul className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                                <li>• MONTEE / DESCENTE</li>
                                <li>• VIRAGE GAUCHE / DROITE</li>
                                <li>• ROTATION G / D</li>
                                <li>• VOL STABLE</li>
                              </ul>
                            </div>
                            <div className="bg-gray-50 dark:bg-gray-700 p-4 rounded">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Ligne 2 - Paramètres quantitatifs</h5>
                              <div className="text-sm text-gray-600 dark:text-gray-400">
                                <p><code>J: [score]</code> - Indice de à-coup</p>
                                <p><code>A: [valeur] m/s²</code> - Norme accélération</p>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Tests recommandés */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Manœuvres de test recommandées</h4>
                          <div className="grid gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Test d'inclinaison</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Inclinaison lente et contrôlée du dispositif
                              </p>
                              <div className="text-xs text-gray-500 dark:text-gray-500">
                                Résultat attendu: "MONTEE" ou "DESCENTE"
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Test de virage</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Basculement latéral vers la gauche ou la droite
                              </p>
                              <div className="text-xs text-gray-500 dark:text-gray-500">
                                Résultat attendu: "VIRAGE GAUCHE" / "VIRAGE DROITE"
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Test de translation</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Glissement horizontal sur surface plane
                              </p>
                              <div className="text-xs text-gray-500 dark:text-gray-500">
                                Résultat attendu: "AVANCE", "RECULE", "GLISSE..."
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Test de rotation</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Rotation rapide autour de l'axe vertical
                              </p>
                              <div className="text-xs text-gray-500 dark:text-gray-500">
                                Résultat attendu: "ROTATION G" ou "ROTATION D"
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Test de stabilité</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Immobilité complète pendant 0.5 seconde
                              </p>
                              <div className="text-xs text-gray-500 dark:text-gray-500">
                                Résultat attendu: "VOL STABLE"
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 8.3 Moniteur série */}
                    <div className="space-y-4">
                      <h3 id={slugify("8.3 Utilisation du moniteur série")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        8.3 Interface de débogage série
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Configuration et utilisation</h4>
                        
                        <div className="space-y-4">
                          <div className="bg-gray-50 dark:bg-gray-700 p-4 rounded">
                            <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Paramètres de communication</h5>
                            <div className="text-sm text-gray-600 dark:text-gray-400">
                              <p><strong>Vitesse:</strong> 9600 bauds</p>
                              <p><strong>Format:</strong> 8N1 (8 bits, aucune parité, 1 bit de stop)</p>
                            </div>
                          </div>
                          
                          <div className="bg-gray-50 dark:bg-gray-700 p-4 rounded">
                            <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-3">Format des données affichées</h5>
                            <div className="bg-gray-900 rounded p-3 text-green-400 font-mono text-sm">
                              <div>MONTEE | R: 15.2 T: -2.1 | Lacet: 1.3 |</div>
                              <div>AX: 0.12 DX: 0.05 AY: -0.03 DY: 0.01 | Jerk: 2.8</div>
                            </div>
                          </div>
                          
                          <div className="grid md:grid-cols-2 gap-4">
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Ligne 1 - Angles</h5>
                              <ul className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                                <li>• <strong>R:</strong> Roulis (°)</li>
                                <li>• <strong>T:</strong> Tangage (°)</li>
                                <li>• <strong>Lacet:</strong> Vitesse angulaire Z (°/s)</li>
                              </ul>
                            </div>
                            <div>
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Ligne 2 - Accélérations</h5>
                              <ul className="text-sm text-gray-600 dark:text-gray-400 space-y-1">
                                <li>• <strong>AX/AY:</strong> Accélération instantanée (g)</li>
                                <li>• <strong>DX/DY:</strong> Écart aux moyennes (g)</li>
                                <li>• <strong>Jerk:</strong> Score de à-coup</li>
                              </ul>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* Démonstration vidéo */}
                    <div className="space-y-4">
                      <h3 className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        8.4 Démonstrations vidéo
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Enregistrements des tests</h4>
                        <p className="text-gray-700 dark:text-gray-300 mb-6">
                          Les vidéos suivantes présentent le système en fonctionnement avec différents types de mouvements 
                          et la réponse correspondante sur l'affichage LCD.
                        </p>
                        
                        <div className="space-y-4">
                          <div className="aspect-video rounded-lg overflow-hidden border">
                            <iframe 
                              src="https://player.vimeo.com/video/1092850492" 
                              width="100%" 
                              height="100%" 
                              frameBorder="0" 
                              allow="autoplay; fullscreen; picture-in-picture" 
                              allowFullScreen 
                              className="w-full h-full"
                            ></iframe>
                          </div>
                          
                          <div className="aspect-video rounded-lg overflow-hidden border">
                            <iframe 
                              src="https://www.youtube.com/embed/U2M5avpgowg" 
                              width="100%" 
                              height="100%" 
                              frameBorder="0" 
                              allow="autoplay; fullscreen; picture-in-picture" 
                              allowFullScreen 
                              className="w-full h-full"
                            ></iframe>
                          </div>
                        </div>
                      </div>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* SECTION 9 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle id="9-résultats-attendus" className="flex items-center gap-2 scroll-mt-24">
                        <CheckCircle className="w-5 h-5 text-gray-600" />
                        9. Résultats attendus et validation
                    </CardTitle>
                    <CardDescription>
                      Critères de validation et scénarios de test de référence
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6">

                    <div className="bg-gray-50 dark:bg-gray-900 p-6 rounded-lg border-l-4 border-gray-400">
                      <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                        Cette section présente les résultats attendus pour la validation du système dans différents 
                        scénarios opérationnels. Chaque test doit produire des réponses cohérentes et reproductibles.
                      </p>
                    </div>

                    {/* Scénarios de test */}
                    <div className="bg-white dark:bg-gray-800 border rounded-lg overflow-hidden">
                      <div className="bg-gray-100 dark:bg-gray-700 px-6 py-4 border-b">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200">
                          Scénarios de test et réponses système
                        </h4>
                      </div>
                      
                      <div className="overflow-x-auto">
                        <table className="w-full">
                          <thead className="bg-gray-50 dark:bg-gray-900">
                            <tr>
                              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                                Scénario de test
                              </th>
                              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                                Affichage LCD Ligne 1
                              </th>
                              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                                Affichage LCD Ligne 2
                              </th>
                              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 dark:text-gray-400 uppercase tracking-wider">
                                Moniteur série (extrait)
                              </th>
                            </tr>
                          </thead>
                          <tbody className="bg-white dark:bg-gray-800 divide-y divide-gray-200 dark:divide-gray-700">
                            <tr>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                Inclinaison vers l'avant
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200">
                                  MONTEE
                                </span>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                J: 5 A: 11.3 m/s²
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <code className="text-xs bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">MONTEE</code>
                              </td>
                            </tr>
                            <tr>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                Virage à droite
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-200">
                                  VIRAGE DROITE
                                </span>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                J: 3 A: 9.5 m/s²
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <code className="text-xs bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">VIRAGE DROITE</code>
                              </td>
                            </tr>
                            <tr>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                Glissement latéral
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-purple-100 text-purple-800 dark:bg-purple-900 dark:text-purple-200">
                                  GLISSE DROITE
                                </span>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                J: 2 A: 9.9 m/s²
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <code className="text-xs bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">GLISSE DROITE</code>
                              </td>
                            </tr>
                            <tr>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                Position de repos
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-200">
                                  VOL STABLE
                                </span>
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900 dark:text-gray-100">
                                J: 0 A: 9.81 m/s²
                              </td>
                              <td className="px-6 py-4 whitespace-nowrap">
                                <code className="text-xs bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">VOL STABLE</code>
                              </td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                    </div>

                    {/* Analyses des résultats */}
                    <div className="space-y-4">
                      <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200">
                        Analyse des métriques
                      </h4>
                      
                      <div className="grid md:grid-cols-2 gap-6">
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-3">Score Jerk (indice de à-coup)</h5>
                          <div className="space-y-3">
                            <div className="flex items-center justify-between">
                              <span className="text-sm text-gray-600 dark:text-gray-400">Repos</span>
                              <span className="text-sm font-medium">J: 0-1</span>
                            </div>
                            <div className="flex items-center justify-between">
                              <span className="text-sm text-gray-600 dark:text-gray-400">Mouvement modéré</span>
                              <span className="text-sm font-medium">J: 2-4</span>
                            </div>
                            <div className="flex items-center justify-between">
                              <span className="text-sm text-gray-600 dark:text-gray-400">Mouvement brusque</span>
                              <span className="text-sm font-medium">J: 5+</span>
                            </div>
                          </div>
                          <div className="mt-4 p-3 bg-gray-50 dark:bg-gray-700 rounded text-sm text-gray-600 dark:text-gray-400">
                            <strong>Note:</strong> Plus la manœuvre est soudaine, plus le pic de Jerk est élevé
                          </div>
                        </div>

                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-3">Norme d'accélération</h5>
                          <div className="space-y-3">
                            <div className="flex items-center justify-between">
                              <span className="text-sm text-gray-600 dark:text-gray-400">Référence théorique</span>
                              <span className="text-sm font-medium">9.81 m/s²</span>
                            </div>
                            <div className="flex items-center justify-between">
                              <span className="text-sm text-gray-600 dark:text-gray-400">Tolérance acceptable</span>
                              <span className="text-sm font-medium">±0.2 m/s²</span>
                            </div>
                            <div className="flex items-center justify-between">
                              <span className="text-sm text-gray-600 dark:text-gray-400">En mouvement</span>
                              <span className="text-sm font-medium">Variable</span>
                            </div>
                          </div>
                          <div className="mt-4 p-3 bg-gray-50 dark:bg-gray-700 rounded text-sm text-gray-600 dark:text-gray-400">
                            <strong>Note:</strong> Convergence vers 1g (9.81 m/s²) au repos
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* Critères de validation */}
                    <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                      <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">
                        Critères de validation système
                      </h4>
                      
                      <div className="space-y-3">
                        <div className="flex items-start gap-3">
                          <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                          <div>
                            <h6 className="font-medium text-gray-700 dark:text-gray-300">Réactivité</h6>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Détection et affichage du mouvement en moins de 200ms
                            </p>
                          </div>
                        </div>
                        
                        <div className="flex items-start gap-3">
                          <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                          <div>
                            <h6 className="font-medium text-gray-700 dark:text-gray-300">Précision</h6>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Classification correcte du type de mouvement dans 95% des cas
                            </p>
                          </div>
                        </div>
                        
                        <div className="flex items-start gap-3">
                          <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                          <div>
                            <h6 className="font-medium text-gray-700 dark:text-gray-300">Stabilité</h6>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Retour à "VOL STABLE" après cessation du mouvement (≤ 0.5s)
                            </p>
                          </div>
                        </div>
                        
                        <div className="flex items-start gap-3">
                          <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                          <div>
                            <h6 className="font-medium text-gray-700 dark:text-gray-300">Reproductibilité</h6>
                            <p className="text-sm text-gray-600 dark:text-gray-400">
                              Réponses identiques pour des mouvements similaires (±5% de variation)
                            </p>
                          </div>
                        </div>
                      </div>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 10 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle id="10-conseils-de-réglage-et-calibration" className="flex items-center gap-2 scroll-mt-24">
                        <Wrench className="w-5 h-5 text-gray-600" />
                        10. Optimisation et calibration avancée
                    </CardTitle>
                    <CardDescription>
                      Techniques d'ajustement pour améliorer les performances et la précision du système
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8">

                    {/* 10.1 Calibration environnementale */}
                    <div className="space-y-4">
                      <h3 id={slugify("10.1 Refaire la calibration selon lenvironnement")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        10.1 Calibration environnementale
                      </h3>
                      
                      <div className="space-y-6">
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Conditions optimales de calibration</h4>
                          
                          <div className="grid gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <div className="flex items-center gap-2 mb-2">
                                <div className="w-2 h-2 bg-red-500 rounded-full"></div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Surface de référence</h5>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Placer le capteur sur une surface parfaitement plane et horizontale, 
                                sans vibrations ni perturbations mécaniques pendant toute la durée de calibration.
                              </p>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <div className="flex items-center gap-2 mb-2">
                                <div className="w-2 h-2 bg-blue-500 rounded-full"></div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Paramètres d'échantillonnage</h5>
                              </div>
                              <div className="space-y-2 text-sm text-gray-600 dark:text-gray-400">
                                <p><strong>Défaut:</strong> 1000 échantillons (≈ 10 secondes)</p>
                                <p><strong>Précision élevée:</strong> 2000+ échantillons (temps de calibration plus long)</p>
                                <p><strong>Test rapide:</strong> 500 échantillons (précision réduite)</p>
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <div className="flex items-center gap-2 mb-2">
                                <div className="w-2 h-2 bg-green-500 rounded-full"></div>
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">Facteurs environnementaux</h5>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Les caractéristiques MEMS varient avec la température. 
                                Recalibrer lors de changements significatifs d'environnement 
                                (extérieur ↔ intérieur, variations &gt; 15°C).
                              </p>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 10.2 Ajustement des seuils */}
                    <div className="space-y-4">
                      <h3 id={slugify("10.2 Ajuster les seuils de détection")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        10.2 Ajustement des seuils de détection
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Paramètres de sensibilité</h4>
                        
                        <div className="space-y-4">
                          <div className="overflow-x-auto">
                            <table className="w-full border-collapse">
                              <thead>
                                <tr className="bg-gray-50 dark:bg-gray-900">
                                  <th className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-left text-sm font-medium text-gray-700 dark:text-gray-300">
                                    Paramètre
                                  </th>
                                  <th className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-left text-sm font-medium text-gray-700 dark:text-gray-300">
                                    Valeur actuelle
                                  </th>
                                  <th className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-left text-sm font-medium text-gray-700 dark:text-gray-300">
                                    Fonction
                                  </th>
                                  <th className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-left text-sm font-medium text-gray-700 dark:text-gray-300">
                                    Ajustement suggéré
                                  </th>
                                </tr>
                              </thead>
                              <tbody>
                                <tr>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    Seuil roulis/tangage
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">±12°</code>
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    Détection inclinaison
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    ±8° (plus sensible)<br/>±15° (moins sensible)
                                  </td>
                                </tr>
                                <tr className="bg-gray-50 dark:bg-gray-900">
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    Seuil vitesse lacet
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">45°/s</code>
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    Détection rotation
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    30°/s (plus sensible)<br/>60°/s (rotations rapides)
                                  </td>
                                </tr>
                                <tr>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    Seuil glissement
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    <code className="bg-gray-100 dark:bg-gray-700 px-2 py-1 rounded">0.15g</code>
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    Translation avant/arrière/latérale
                                  </td>
                                  <td className="border border-gray-200 dark:border-gray-700 px-4 py-3 text-sm">
                                    0.10g (mouvements faibles)<br/>0.20g (moins de faux positifs)
                                  </td>
                                </tr>
                              </tbody>
                            </table>
                          </div>
                          
                          <div className="bg-blue-50 dark:bg-blue-900 p-4 rounded-lg border-l-4 border-blue-500">
                            <h5 className="font-medium text-blue-800 dark:text-blue-200 mb-2">Méthode de test</h5>
                            <p className="text-sm text-blue-700 dark:text-blue-300">
                              Pour chaque seuil, effectuer des mouvements lents puis rapides et observer 
                              la réactivité sur le moniteur série avant modification du code source.
                            </p>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 10.3 Configuration DLPF */}
                    <div className="space-y-4">
                      <h3 id={slugify("10.3 Régler le filtre passe-bas DLPF")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        10.3 Optimisation du filtre numérique
                      </h3>
                      
                      <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                        <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Configuration du DLPF (Digital Low Pass Filter)</h4>
                        
                        <div className="space-y-4">
                          <div className="grid md:grid-cols-3 gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <div className="flex items-center justify-between mb-2">
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">20 Hz</h5>
                                <span className="text-xs bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-200 px-2 py-1 rounded">
                                  Actuel
                                </span>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Configuration optimale pour applications lentes (robots, maquettes)
                              </p>
                              <ul className="text-xs text-gray-500 dark:text-gray-500 space-y-1">
                                <li>• Suppression maximale du bruit</li>
                                <li>• Réponse lissée</li>
                                <li>• Recommandé pour ce projet</li>
                              </ul>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <div className="flex items-center justify-between mb-2">
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">42 Hz</h5>
                                <span className="text-xs bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-200 px-2 py-1 rounded">
                                  Équilibré
                                </span>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Compromis entre réactivité et filtrage
                              </p>
                              <ul className="text-xs text-gray-500 dark:text-gray-500 space-y-1">
                                <li>• Réactivité modérée</li>
                                <li>• Bruit résiduel acceptable</li>
                                <li>• Applications polyvalentes</li>
                              </ul>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <div className="flex items-center justify-between mb-2">
                                <h5 className="font-medium text-gray-700 dark:text-gray-300">98 Hz</h5>
                                <span className="text-xs bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-200 px-2 py-1 rounded">
                                  Rapide
                                </span>
                              </div>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Réactivité maximale aux mouvements rapides
                              </p>
                              <ul className="text-xs text-gray-500 dark:text-gray-500 space-y-1">
                                <li>• Réponse instantanée</li>
                                <li>• Plus de bruit haute fréquence</li>
                                <li>• Applications dynamiques</li>
                              </ul>
                            </div>
                          </div>
                          
                          <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg">
                            <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Modification du code</h5>
                            <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
                              Dans la fonction <code>setup()</code>, remplacer la ligne :
                            </p>
                            <div className="bg-gray-900 rounded p-3 text-gray-100 text-sm font-mono mb-2">
                              <code>capteurMouvement.setDLPFMode(MPU6050_DLPF_BW_42);</code>
                            </div>
                            <p className="text-xs text-gray-500 dark:text-gray-500">
                              Valeurs disponibles : MPU6050_DLPF_BW_5, _10, _20, _42, _98, _188, _256
                            </p>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 10.4 Filtre complémentaire vs DMP */}
                    <div className="space-y-4">
                      <h3 id={slugify("10.4 Filtre complémentaire vs DMP")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        10.4 Stratégies de fusion de données
                      </h3>
                      
                      <div className="grid md:grid-cols-2 gap-6">
                        {/* Filtre logiciel */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">
                            Filtre complémentaire (logiciel)
                          </h4>
                          
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-1">Coefficient α actuel</h5>
                              <div className="flex items-center justify-between">
                                <code className="text-sm">0.94</code>
                                <span className="text-xs text-gray-500">94% gyro, 6% accéléro</span>
                              </div>
                            </div>
                            
                            <div className="space-y-2">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300">Ajustements possibles</h5>
                              <div className="text-sm space-y-1">
                                <div className="flex justify-between">
                                  <span>α = 0.98</span>
                                  <span className="text-gray-500">Plus de stabilité long terme</span>
                                </div>
                                <div className="flex justify-between">
                                  <span>α = 0.90</span>
                                  <span className="text-gray-500">Plus de réactivité</span>
                                </div>
                              </div>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-900 p-3 rounded">
                              <h6 className="text-sm font-medium text-gray-800 dark:text-gray-200 mb-1">Avantages</h6>
                              <ul className="text-xs text-gray-700 dark:text-gray-300 space-y-1">
                                <li>• Contrôle total sur l'algorithme</li>
                                <li>• Simple à comprendre et modifier</li>
                                <li>• Faible charge CPU</li>
                              </ul>
                            </div>
                          </div>
                        </div>

                        {/* DMP */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">
                            DMP (Digital Motion Processor)
                          </h4>
                          
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-700 p-3 rounded">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-1">Principe</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Chargement du microcode dans le MPU-6050 pour fusion automatique 
                                accéléromètre + gyroscope
                              </p>
                            </div>
                            
                            <div className="bg-blue-50 dark:bg-blue-900 p-3 rounded">
                              <h6 className="text-sm font-medium text-blue-800 dark:text-blue-200 mb-1">Avantages</h6>
                              <ul className="text-xs text-blue-700 dark:text-blue-300 space-y-1">
                                <li>• Résultats pré-filtrés et optimisés</li>
                                <li>• Moins de calculs sur Arduino</li>
                                <li>• Algorithmes propriétaires InvenSense</li>
                              </ul>
                            </div>
                            
                            <div className="bg-red-50 dark:bg-red-900 p-3 rounded">
                              <h6 className="text-sm font-medium text-red-800 dark:text-red-200 mb-1">Inconvénients</h6>
                              <ul className="text-xs text-red-700 dark:text-red-300 space-y-1">
                                <li>• Setup plus complexe</li>
                                <li>• Chargement de firmware requis</li>
                                <li>• Moins de contrôle direct</li>
                                <li>• Documentation limitée</li>
                              </ul>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 11 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle id="11-limitations-et-pistes-damélioration" className="flex items-center gap-2 scroll-mt-24">
                        <AlertTriangle className="w-5 h-5 text-gray-600" />
                        11. Analyse critique et perspectives d'évolution
                    </CardTitle>
                    <CardDescription>
                      Évaluation des limitations actuelles et propositions d'améliorations techniques
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8">

                    <div className="bg-gray-50 dark:bg-gray-900 p-6 rounded-lg border-l-4 border-gray-400">
                      <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                        Bien que le système actuel réponde aux exigences du Test 1, cette analyse identifie 
                        les limitations techniques et propose des axes d'amélioration pour optimiser les performances 
                        et la robustesse de la solution.
                      </p>
                    </div>

                    {/* 11.1 Limitations actuelles */}
                    <div className="space-y-4">
                      <h3 id={slugify("11.1 Limitations actuelles")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        11.1 Contraintes techniques identifiées
                      </h3>
                      
                      <div className="space-y-4">
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Limitations logicielles</h4>
                          
                          <div className="space-y-4">
                            <div className="border-l-4 border-gray-500 bg-gray-50 dark:bg-gray-900 p-4 rounded">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Gestion mémoire (String Arduino)</h5>
                              <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <p><strong>Problème:</strong> Fragmentation de la heap avec utilisation intensive des objets String</p>
                                <p><strong>Conséquence:</strong> Risque de plantage système sur le long terme</p>
                                <p><strong>Impact:</strong> Fonctionnement continu limité à quelques heures</p>
                              </div>
                            </div>
                            
                            <div className="border-l-4 border-gray-500 bg-gray-50 dark:bg-gray-900 p-4 rounded">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Performance d'affichage LCD</h5>
                              <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <p><strong>Problème:</strong> Protocole HD44780 lent (millisecondes par caractère)</p>
                                <p><strong>Conséquence:</strong> Fréquence d'affichage limitée à ~10 Hz</p>
                                <p><strong>Impact:</strong> Latence perceptible lors de mouvements rapides</p>
                              </div>
                            </div>
                            
                            <div className="border-l-4 border-gray-500 bg-gray-50 dark:bg-gray-900 p-4 rounded">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Architecture interruptions</h5>
                              <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <p><strong>Problème:</strong> Lecture périodique via delay(), pas d'interruptions</p>
                                <p><strong>Conséquence:</strong> Échantillonnage non optimal, dérive temporelle</p>
                                <p><strong>Impact:</strong> Précision temporelle limitée pour fusion de données</p>
                              </div>
                            </div>
                          </div>
                        </div>

                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Limitations matérielles</h4>
                          
                          <div className="space-y-4">
                            <div className="border-l-4 border-gray-500 bg-gray-50 dark:bg-gray-900 p-4 rounded">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Fiabilité du montage</h5>
                              <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <p><strong>Problème:</strong> Connexions breadboard sensibles aux vibrations</p>
                                <p><strong>Conséquence:</strong> Faux contacts, perturbations I²C</p>
                                <p><strong>Impact:</strong> Instabilité opérationnelle, erreurs de communication</p>
                              </div>
                            </div>
                            
                            <div className="border-l-4 border-gray-500 bg-gray-50 dark:bg-gray-900 p-4 rounded">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Immunité au bruit électrique</h5>
                              <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <p><strong>Problème:</strong> Bus I²C sensible aux interférences électromagnétiques</p>
                                <p><strong>Conséquence:</strong> Corruption de données, lectures erronées</p>
                                <p><strong>Impact:</strong> Dégradation de la précision en environnement perturbé</p>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 11.2 Pistes d'amélioration */}
                    <div className="space-y-4">
                      <h3 id={slugify("11.2 Pistes damélioration")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        11.2 Stratégies d'optimisation proposées
                      </h3>
                      
                      <div className="space-y-6">
                        {/* Améliorations logicielles */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Optimisations logicielles</h4>
                          
                          <div className="grid gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-gray-900 rounded p-4">
                              <div className="flex items-start gap-3">
                                <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400 flex-shrink-0 mt-0.5" />
                                <div>
                                  <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Gestion mémoire optimisée</h5>
                                  <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                    <p><strong>Solution:</strong> Remplacement des String par char[] et snprintf()</p>
                                    <p><strong>Bénéfice:</strong> Élimination de la fragmentation mémoire</p>
                                    <p><strong>Complexité:</strong> Faible - modification directe du code existant</p>
                                  </div>
                                </div>
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-gray-900 rounded p-4">
                              <div className="flex items-start gap-3">
                                <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400 flex-shrink-0 mt-0.5" />
                                <div>
                                  <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Intégration du DMP</h5>
                                  <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                    <p><strong>Solution:</strong> Utilisation du Digital Motion Processor interne</p>
                                    <p><strong>Bénéfice:</strong> Fusion de données embarquée, précision accrue</p>
                                    <p><strong>Complexité:</strong> Modérée - nécessite chargement de firmware</p>
                                  </div>
                                </div>
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-gray-900 rounded p-4">
                              <div className="flex items-start gap-3">
                                <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400 flex-shrink-0 mt-0.5" />
                                <div>
                                  <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Architecture événementielle</h5>
                                  <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                    <p><strong>Solution:</strong> Utilisation des interruptions INT du MPU-6050</p>
                                    <p><strong>Bénéfice:</strong> Échantillonnage synchrone, élimination des delay()</p>
                                    <p><strong>Complexité:</strong> Modérée - restructuration de l'architecture</p>
                                  </div>
                                </div>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Améliorations matérielles */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Évolutions matérielles</h4>
                          
                          <div className="grid gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-gray-900 rounded p-4">
                              <div className="flex items-start gap-3">
                                <Wrench className="w-5 h-5 text-gray-600 dark:text-gray-400 flex-shrink-0 mt-0.5" />
                                <div>
                                  <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Affichage haute performance</h5>
                                  <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                    <p><strong>Solution:</strong> Remplacement par OLED I²C (SSD1306)</p>
                                    <p><strong>Bénéfice:</strong> Taux de rafraîchissement 60+ Hz, graphiques possibles</p>
                                    <p><strong>Coût:</strong> +5-10€, compatible I²C existant</p>
                                  </div>
                                </div>
                              </div>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 bg-gray-50 dark:bg-gray-900 rounded p-4">
                              <div className="flex items-start gap-3">
                                <Wrench className="w-5 h-5 text-gray-600 dark:text-gray-400 flex-shrink-0 mt-0.5" />
                                <div>
                                  <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Conception PCB</h5>
                                  <div className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                    <p><strong>Solution:</strong> Circuit imprimé dédié avec plans de masse</p>
                                    <p><strong>Bénéfice:</strong> Fiabilité électrique, immunité au bruit, compacité</p>
                                    <p><strong>Coût:</strong> 20-50€ selon quantité, délai de fabrication</p>
                                  </div>
                                </div>
                              </div>
                            </div>
                            
                            <div className="border border-blue-200 dark:border-blue-700 bg-blue-50 dark:bg-blue-900 rounded p-4">
                              <div className="flex items-start gap-3">
                                <Wrench className="w-5 h-5 text-blue-600 dark:text-blue-400 flex-shrink-0 mt-0.5" />
                                <div>
                                  <h5 className="font-medium text-blue-800 dark:text-blue-200 mb-2">Enregistrement de données</h5>
                                  <div className="text-sm text-blue-700 dark:text-blue-300 space-y-1">
                                    <p><strong>Solution:</strong> Module carte SD pour logging des mesures</p>
                                    <p><strong>Bénéfice:</strong> Analyse post-test, validation, traçabilité</p>
                                    <p><strong>Coût:</strong> +3-5€, intégration logicielle requise</p>
                                  </div>
                                </div>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Évaluation technique */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Évaluation de l'implémentation actuelle</h4>
                          
                          <div className="space-y-4">
                            <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg border-l-4 border-gray-500">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Points forts identifiés</h5>
                              <ul className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <li>• Réponse aux exigences du Test 1 du TRC 2025</li>
                                <li>• Architecture logicielle simple et compréhensible</li>
                                <li>• Coût matériel réduit et composants accessibles</li>
                                <li>• Documentation complète pour reproduction</li>
                                <li>• Calibration automatique fonctionnelle</li>
                              </ul>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg border-l-4 border-gray-400">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Compromis techniques acceptés</h5>
                              <ul className="text-sm text-gray-700 dark:text-gray-300 space-y-1">
                                <li>• Fréquence d'affichage limitée mais suffisante pour l'usage</li>
                                <li>• Montage breadboard adapté au contexte éducatif</li>
                                <li>• Filtre logiciel simple mais efficace</li>
                                <li>• Précision acceptable pour les objectifs du test</li>
                              </ul>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-900 p-4 rounded-lg border-l-4 border-gray-600">
                              <h5 className="font-medium text-gray-800 dark:text-gray-200 mb-2">Validation opérationnelle</h5>
                              <div className="text-sm text-gray-700 dark:text-gray-300">
                                <p>
                                  Le système développé répond pleinement aux critères du Test 1 Électronique 
                                  du Tekbot Robotics Challenge 2025, avec une détection fiable des mouvements 
                                  et un affichage en temps réel des paramètres inertiels.
                                </p>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 12 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle id="12-références-et-annexes" className="flex items-center gap-2 scroll-mt-24">
                        <FileText className="w-5 h-5 text-gray-600" />
                        12. Références bibliographiques et annexes
                    </CardTitle>
                    <CardDescription>
                      Documentation technique, sources de référence et ressources complémentaires
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8">

                    {/* 12.1 Références */}
                    <div className="space-y-4">
                      <h3 id={slugify("12.1 Références")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        12.1 Références techniques
                      </h3>
                      
                      <div className="space-y-6">
                        {/* Documentation composants */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Documentation des composants</h4>
                          
                          <div className="space-y-4">
                            <div className="border-l-4 border-blue-500 bg-blue-50 dark:bg-blue-900 p-4 rounded">
                              <h5 className="font-medium text-blue-800 dark:text-blue-200 mb-2">MPU-6050 (InvenSense/TDK)</h5>
                              <div className="space-y-2 text-sm">
                                <p className="text-blue-700 dark:text-blue-300">
                                  <strong>Titre:</strong> "MPU-6000 and MPU-6050 Product Specification Revision 3.4"
                                </p>
                                <p className="text-blue-700 dark:text-blue-300">
                                  <strong>Éditeur:</strong> InvenSense Inc., 2013
                                </p>
                                <a 
                                  href="https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf" 
                                  target="_blank" 
                                  rel="noopener noreferrer"
                                  className="inline-flex items-center gap-1 text-blue-600 dark:text-blue-400 hover:text-blue-800 dark:hover:text-blue-200"
                                >
                                  <span>Documentation officielle</span>
                                  <ExternalLink className="w-3 h-3" />
                                </a>
                              </div>
                            </div>
                            
                            <div className="border-l-4 border-green-500 bg-green-50 dark:bg-green-900 p-4 rounded">
                              <h5 className="font-medium text-green-800 dark:text-green-200 mb-2">Régulateur de tension 7805</h5>
                              <div className="space-y-2 text-sm">
                                <p className="text-green-700 dark:text-green-300">
                                  <strong>Titre:</strong> "LM7805 Series Positive Voltage Regulator"
                                </p>
                                <p className="text-green-700 dark:text-green-300">
                                  <strong>Éditeur:</strong> ON Semiconductor
                                </p>
                                <a 
                                  href="https://www.onsemi.com/pdf/datasheet/lm7805-d.pdf" 
                                  target="_blank" 
                                  rel="noopener noreferrer"
                                  className="inline-flex items-center gap-1 text-green-600 dark:text-green-400 hover:text-green-800 dark:hover:text-green-200"
                                >
                                  <span>Datasheet technique</span>
                                  <ExternalLink className="w-3 h-3" />
                                </a>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Bibliothèques logicielles */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Bibliothèques et frameworks</h4>
                          
                          <div className="space-y-4">
                            <div className="border-l-4 border-purple-500 bg-purple-50 dark:bg-purple-900 p-4 rounded">
                              <h5 className="font-medium text-purple-800 dark:text-purple-200 mb-2">I2Cdev Library</h5>
                              <div className="space-y-2 text-sm">
                                <p className="text-purple-700 dark:text-purple-300">
                                  <strong>Auteur:</strong> Jeff Rowberg
                                </p>
                                <p className="text-purple-700 dark:text-purple-300">
                                  <strong>Description:</strong> Bibliothèque I²C pour Arduino avec support MPU-6050
                                </p>
                                <a 
                                  href="https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050" 
                                  target="_blank" 
                                  rel="noopener noreferrer"
                                  className="inline-flex items-center gap-1 text-purple-600 dark:text-purple-400 hover:text-purple-800 dark:hover:text-purple-200"
                                >
                                  <span>Repository GitHub</span>
                                  <ExternalLink className="w-3 h-3" />
                                </a>
                              </div>
                            </div>
                            
                            <div className="border-l-4 border-orange-500 bg-orange-50 dark:bg-orange-900 p-4 rounded">
                              <h5 className="font-medium text-orange-800 dark:text-orange-200 mb-2">LiquidCrystal Library</h5>
                              <div className="space-y-2 text-sm">
                                <p className="text-orange-700 dark:text-orange-300">
                                  <strong>Éditeur:</strong> Arduino LLC
                                </p>
                                <p className="text-orange-700 dark:text-orange-300">
                                  <strong>Description:</strong> Interface pour écrans LCD HD44780
                                </p>
                                <a 
                                  href="https://www.arduino.cc/en/Reference/LiquidCrystal" 
                                  target="_blank" 
                                  rel="noopener noreferrer"
                                  className="inline-flex items-center gap-1 text-orange-600 dark:text-orange-400 hover:text-orange-800 dark:hover:text-orange-200"
                                >
                                  <span>Documentation Arduino</span>
                                  <ExternalLink className="w-3 h-3" />
                                </a>
                              </div>
                            </div>
                          </div>
                        </div>

                        {/* Outils de développement */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">Environnement de développement</h4>
                          
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Arduino IDE</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Environnement de développement intégré pour microcontrôleurs Arduino
                              </p>
                              <a 
                                href="https://www.arduino.cc/en/software" 
                                target="_blank" 
                                rel="noopener noreferrer"
                                className="inline-flex items-center gap-1 text-blue-600 dark:text-blue-400 hover:text-blue-800 dark:hover:text-blue-200 text-sm"
                              >
                                <span>Téléchargement</span>
                                <ExternalLink className="w-3 h-3" />
                              </a>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">KiCad EDA</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Suite de conception électronique open-source pour schémas et PCB
                              </p>
                              <a 
                                href="https://www.kicad.org/" 
                                target="_blank" 
                                rel="noopener noreferrer"
                                className="inline-flex items-center gap-1 text-blue-600 dark:text-blue-400 hover:text-blue-800 dark:hover:text-blue-200 text-sm"
                              >
                                <span>Site officiel</span>
                                <ExternalLink className="w-3 h-3" />
                              </a>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                    {/* 12.2 Annexes */}
                    <div className="space-y-4">
                      <h3 id={slugify("12.2 Annexes")} className="text-xl font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                        12.2 Ressources complémentaires
                      </h3>
                      
                      <div className="space-y-6">
                        {/* Code source */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">A. Code source et fichiers projet</h4>
                          
                          <div className="space-y-3">
                            <div className="bg-gray-50 dark:bg-gray-700 p-4 rounded border-l-4 border-gray-400">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Programme principal</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Code source complet avec commentaires et documentation inline
                              </p>
                              <div className="flex items-center gap-2">
                                <code className="bg-gray-100 dark:bg-gray-600 px-2 py-1 rounded text-sm">
                                  Test1_MPU6050.ino
                                </code>
                                <a 
                                  href="https://github.com/votre-equipe/tekbot-trc2025/blob/main/Test1/Test1_MPU6050.ino" 
                                  target="_blank" 
                                  rel="noopener noreferrer"
                                  className="inline-flex items-center gap-1 text-blue-600 dark:text-blue-400 hover:text-blue-800 dark:hover:text-blue-200 text-sm"
                                >
                                  <span>GitHub Repository</span>
                                  <ExternalLink className="w-3 h-3" />
                                </a>
                              </div>
                            </div>
                            
                            <div className="bg-gray-50 dark:bg-gray-700 p-4 rounded border-l-4 border-gray-400">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">Schémas électriques</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                                Fichiers KiCad pour reproduction et modification du circuit
                              </p>
                              <code className="bg-gray-100 dark:bg-gray-600 px-2 py-1 rounded text-sm">
                                Kicad/Test1_MPU6050_Schema.kicad_sch
                              </code>
                            </div>
                          </div>
                        </div>

                        {/* Glossaire technique */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">B. Glossaire technique</h4>
                          
                          <div className="overflow-x-auto">
                            <table className="w-full border-collapse">
                              <thead>
                                <tr className="bg-gray-50 dark:bg-gray-900 border-b">
                                  <th className="text-left p-3 font-medium text-gray-700 dark:text-gray-300">Terme</th>
                                  <th className="text-left p-3 font-medium text-gray-700 dark:text-gray-300">Définition</th>
                                  <th className="text-left p-3 font-medium text-gray-700 dark:text-gray-300">Contexte</th>
                                </tr>
                              </thead>
                              <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                                <tr>
                                  <td className="p-3 font-medium text-gray-900 dark:text-gray-100">LSB</td>
                                  <td className="p-3 text-sm text-gray-600 dark:text-gray-400">
                                    Least Significant Bit - Unité numérique élémentaire des convertisseurs ADC
                                  </td>
                                  <td className="p-3 text-sm text-gray-500 dark:text-gray-500">Conversion analogique-numérique</td>
                                </tr>
                                <tr className="bg-gray-50 dark:bg-gray-900">
                                  <td className="p-3 font-medium text-gray-900 dark:text-gray-100">DLPF</td>
                                  <td className="p-3 text-sm text-gray-600 dark:text-gray-400">
                                    Digital Low Pass Filter - Filtre numérique passe-bas intégré au capteur
                                  </td>
                                  <td className="p-3 text-sm text-gray-500 dark:text-gray-500">Traitement du signal</td>
                                </tr>
                                <tr>
                                  <td className="p-3 font-medium text-gray-900 dark:text-gray-100">DMP</td>
                                  <td className="p-3 text-sm text-gray-600 dark:text-gray-400">
                                    Digital Motion Processor - Coprocesseur interne du MPU pour fusion de données
                                  </td>
                                  <td className="p-3 text-sm text-gray-500 dark:text-gray-500">Architecture embarquée</td>
                                </tr>
                                <tr className="bg-gray-50 dark:bg-gray-900">
                                  <td className="p-3 font-medium text-gray-900 dark:text-gray-100">Jerk</td>
                                  <td className="p-3 text-sm text-gray-600 dark:text-gray-400">
                                    Dérivée de l'accélération par rapport au temps (m/s³)
                                  </td>
                                  <td className="p-3 text-sm text-gray-500 dark:text-gray-500">Mécanique / Détection de mouvement</td>
                                </tr>
                                <tr>
                                  <td className="p-3 font-medium text-gray-900 dark:text-gray-100">IMU</td>
                                  <td className="p-3 text-sm text-gray-600 dark:text-gray-400">
                                    Inertial Measurement Unit - Unité de mesure inertielle (accéléromètre + gyroscope)
                                  </td>
                                  <td className="p-3 text-sm text-gray-500 dark:text-gray-500">Capteurs inertiels</td>
                                </tr>
                                <tr className="bg-gray-50 dark:bg-gray-900">
                                  <td className="p-3 font-medium text-gray-900 dark:text-gray-100">I²C</td>
                                  <td className="p-3 text-sm text-gray-600 dark:text-gray-400">
                                    Inter-Integrated Circuit - Protocole de communication série synchrone
                                  </td>
                                  <td className="p-3 text-sm text-gray-500 dark:text-gray-500">Communication numérique</td>
                                </tr>
                              </tbody>
                            </table>
                          </div>
                        </div>

                        {/* Standards et normes */}
                        <div className="bg-white dark:bg-gray-800 border rounded-lg p-6">
                          <h4 className="text-lg font-medium text-gray-800 dark:text-gray-200 mb-4">C. Standards et normes applicables</h4>
                          
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">ISO 14813-1</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Systèmes de transport intelligents - Architecture de référence
                              </p>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">IEEE 1451.4</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Interface mixte pour capteurs intelligents
                              </p>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">IPC-2221</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Standard de conception PCB pour circuits imprimés
                              </p>
                            </div>
                            
                            <div className="border border-gray-200 dark:border-gray-700 rounded p-4">
                              <h5 className="font-medium text-gray-700 dark:text-gray-300 mb-2">JEDEC JEP95</h5>
                              <p className="text-sm text-gray-600 dark:text-gray-400">
                                Spécifications pour capteurs MEMS
                              </p>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>

                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <div className="text-center text-muted-foreground pt-4">
                <p><strong>Fin de la documentation du Test 1 Electronique – Tekbot Robotics Challenge 2025</strong></p>
              </div>

              {/* Navigation footer */}
              <div className="flex items-center justify-between pt-8 border-t">
                 {/* NOTE: Mettez à jour les liens de navigation */}
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