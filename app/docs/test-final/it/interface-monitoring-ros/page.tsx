"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, BookOpen, GitBranch, PlayCircle, FolderArchive, Target, CheckCircle, BarChart2, AlertTriangle, ExternalLink, FileText, Settings, Layers, Bot, Cpu, Wifi, History, Rocket
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function ConveyorSupervisionDocPage() {
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
                      Documentation
                    </Link>
                    <span>/</span>
                    <span>Projet de Supervision de Convoyeur Intelligent</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Settings className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Projet de Supervision de Convoyeur Intelligent</h1>
                      <p className="text-muted-foreground">Documentation technique de l'architecture, des fonctionnalités et des perspectives.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">ROS 2</Badge>
                    <Badge variant="outline">Electron</Badge>
                    <Badge variant="outline">React</Badge>
                    <Badge variant="outline">Full-Stack</Badge>
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
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Philosophie du Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      L'objectif est de construire un écosystème de contrôle industriel <strong>robuste, intuitif et évolutif</strong>. Au-delà du simple tri d'objets, le projet explore l'intégration d'une interface homme-machine (IHM) moderne avec un framework robotique standard de l'industrie (ROS2), en posant les bases d'un système maintenable et extensible.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <GitBranch className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      🏛️ Architecture Technique Détaillée
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <p>Le système est découplé en trois blocs principaux :</p>
                    <ul className="list-decimal pl-5">
                      <li><strong>Le Cerveau (Backend ROS2) :</strong> Gère la logique métier.</li>
                      <li><strong>L'Interface (Frontend Electron/React) :</strong> Offre visualisation et interaction.</li>
                      <li><strong>Le Pont de Communication (`rosbridge`) :</strong> Le traducteur entre les deux mondes.</li>
                    </ul>

                    <h3 className="font-semibold text-lg">1. Le Cerveau : Backend ROS2 (`ros2_ws`)</h3>
                    <ul className="list-disc pl-5 text-sm">
                      <li><strong>`convoyeur_controller` :</strong> Nœud orchestrateur, fait le lien entre l'IHM et le matériel.</li>
                      <li><strong>`hardware_simulator_node` :</strong> Simule le comportement du matériel, interchangeable avec les vrais drivers.</li>
                      <li><strong>`custom_interfaces` :</strong> Définit des messages, services et actions custom pour une communication fortement typée.</li>
                    </ul>

                    <h3 className="font-semibold text-lg">2. L'Interface : Frontend Electron/React (`electron_convoyeur`)</h3>
                     <ul className="list-disc pl-5 text-sm">
                      <li><strong>Electron :</strong> Conteneur pour l'application de bureau native.</li>
                      <li><strong>React :</strong> Bibliothèque pour construire l'interface en composants fonctionnels.</li>
                      <li><strong>Gestion d'état global (`React Context`) :</strong> Centralise l'instance `roslibjs` et l'état de la connexion.</li>
                    </ul>

                    <h3 className="font-semibold text-lg">3. Le Pont de Communication : `rosbridge_server`</h3>
                    <p>Expose l'écosystème ROS2 à travers une API WebSocket, permettant au frontend de communiquer via des messages JSON traduits en commandes ROS2.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Layers className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      GUI : Guide Détaillé des Pages de l'Interface
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <h3 className="font-semibold">1. Page du Tableau de Bord (Dashboard)</h3>
                    <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/FinalTest_Conv/ass/dashboard.png" alt="Dashboard" width={700} height={400} className="rounded-md border"/>
                    <p className="text-sm">Fournit une vue d'ensemble de l'état du système avec des widgets de statut en temps réel, un graphique d'activité moteur (avec Recharts) et des compteurs de tri, le tout mis à jour via un `RosContext`.</p>
                    
                    <h3 className="font-semibold">2. Page de Calibration</h3>
                    <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/FinalTest_Conv/ass/calibrate.png" alt="Calibration" width={700} height={400} className="rounded-md border"/>
                    <p className="text-sm">Permet d'adapter le logiciel au monde réel. La capture d'empreinte couleur se fait via un service ROS. La synchronisation des données entre modules utilise le patron "remontée d'état" de React.</p>
                    
                    <h3 className="font-semibold">3. Page de Contrôle en Direct (Live Control)</h3>
                    <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/FinalTest_Conv/ass/livecontrol.png" alt="Live Control" width={700} height={400} className="rounded-md border"/>
                    <p className="text-sm">Offre un contrôle direct et une visualisation 3D ("jumeau numérique") avec `@react-three/fiber`, ainsi qu'un terminal interactif basé sur `xterm.js`.</p>
                    
                    <h3 className="font-semibold">4. Page de Supervision ROS</h3>
                    <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/FinalTest_Conv/ass/ros_supervision.png" alt="Supervision ROS" width={700} height={400} className="rounded-md border"/>
                    <p className="text-sm">Fournit une vue de débogage de bas niveau listant les topics, services et nœuds actifs, en utilisant les fonctions natives de `roslibjs`.</p>

                    <h3 className="font-semibold">5. Page des Logs</h3>
                    <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/FinalTest_Conv/ass/logs.png" alt="Logs" width={700} height={400} className="rounded-md border"/>
                    <p className="text-sm">Fournit une traçabilité complète des événements système, gérée par un `LogContext` React, avec une capacité de filtrage par origine.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Utilisation et Lancement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <h3 className="font-semibold">A. Prérequis</h3>
                    <ul className="list-disc pl-5 text-sm"><li>ROS 2 Humble, Node.js, npm, Python 3.</li></ul>
                    <h3 className="font-semibold">B. Configuration de l'Espace de Travail ROS 2</h3>
                    <p className="text-sm">Cloner le dépôt, compiler avec `colcon build`, et sourcer l'environnement avec `source install/setup.bash`.</p>
                    <h3 className="font-semibold">C. Lancement du Système (4 terminaux nécessaires)</h3>
                    <ol className="list-decimal pl-5 text-sm">
                      <li>Terminal 1 : `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`</li>
                      <li>Terminal 2 : `ros2 run convoyeur_controller hardware_simulator_node`</li>
                      <li>Terminal 3 : `ros2 run convoyeur_controller convoyeur_node`</li>
                      <li>Terminal 4 : `cd electron_convoyeur && npm install && npm start`</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Rocket className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      🔭 Évolution et Perspectives : La Vision Micro-ROS
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <p>Une évolution majeure envisagée est l'intégration de **Micro-ROS** pour décentraliser l'intelligence sur des microcontrôleurs (ex: ESP32), rendant chaque composant matériel (moteur, capteur) un nœud ROS ultra-léger.</p>
                    <p className="font-semibold">Avantages :</p>
                    <ul className="list-disc pl-5 text-sm"><li>Modularité extrême, robustesse accrue, performance améliorée.</li></ul>
                    <p className="font-semibold">Défis :</p>
                    <p className="text-sm">L'implémentation a été écartée par manque de temps en raison de la complexité de l'environnement de développement (toolchain, agent Micro-ROS, débogage).</p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <GitBranch className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      🐙 Gestion des Versions avec Git & GitHub
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p>Le projet est versionné sur deux branches distinctes pour séparer le frontend et le backend :</p>
                    <ul className="list-disc pl-5 text-sm">
                      <li>`electron-app` : Code de l'interface utilisateur.</li>
                      <li>`ros-workspace` : Code du système robotique.</li>
                    </ul>
                    <Button asChild variant="link" className="p-0 h-auto mt-2">
                      <Link href="https://github.com/hantanHugues/convoyeur-ROS" target="_blank" rel="noopener noreferrer">
                        Dépôt Git ici <ExternalLink className="w-3 h-3 ml-1"/>
                      </Link>
                    </Button>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* Pied de page de navigation */}
              <div className="flex items-center justify-between pt-8 border-t">
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