"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Target, CheckCircle, Layers, Settings, FileText, BookOpen, ExternalLink, List, FolderArchive, BarChart2
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function FinalTestDocPage() {
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
                    <span>Test Final</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Settings className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">📝 Documentation - Test Final : Système de Contrôle pour Convoyeur de Tri</h1>
                      <p className="text-muted-foreground">Date : 26 Juin 2025</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Test Final</Badge>
                    <Badge variant="outline">Électronique</Badge>
                    <Badge variant="outline">PCB</Badge>
                    <Badge variant="outline">Firmware</Badge>
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
                      <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      📋 Table des matières
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-blue-600 dark:text-blue-400">
                        <li><a href="#1-contexte--objectifs" className="hover:underline">Contexte & Objectifs</a></li>
                        <li><a href="#2-spécifications--livrables" className="hover:underline">Spécifications & Livrables</a></li>
                        <li><a href="#3-processus--workflow" className="hover:underline">Processus & Workflow</a></li>
                        <li><a href="#4-tâches--étapes" className="hover:underline">Tâches & Étapes</a></li>
                        <li><a href="#5-tests--validation" className="hover:underline">Tests & Validation</a></li>
                        <li><a href="#6-fichiers-du-projet" className="hover:underline">Fichiers du Projet</a></li>
                        <li><a href="#7-présentation-des-résultats" className="hover:underline">Présentation des Résultats</a></li>
                        <li><a href="#8-ressources--références" className="hover:underline">Ressources & Références</a></li>
                        <li><a href="#9-annexes-techniques-détails" className="hover:underline">Annexes Techniques (Détails)</a></li>
                        <li><a href="#10-conclusion" className="hover:underline">Conclusion</a></li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <AnimatedSection animation="fade-up" delay={50}>
                <Card id="1-contexte--objectifs" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      1. Contexte & Objectifs
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 text-gray-700 dark:text-gray-300 leading-relaxed space-y-4">
                    <p><strong>Test Final – Système de Contrôle pour Convoyeur de Tri</strong><br/>Ce test final du <strong>TEKBOT Robotics Challenge 2025</strong> est une épreuve de synthèse multidisciplinaire. Notre rôle, en tant qu'équipe électronique, est de concevoir et de réaliser le "cerveau" et le "système nerveux" d'un convoyeur de tri automatisé.</p>
                    <p>Le projet consiste à développer un système électronique complet capable de piloter un convoyeur pour trier des objets (cubes de 30mm) en fonction de leur couleur (Vert, Jaune, Rouge, Bleu) et de fournir les données de tri en temps réel à une API web.</p>
                    <p><strong>Objectifs Spécifiques de l'Équipe Électronique :</strong></p>
                    <ul className="list-disc pl-5 space-y-2">
                      <li><strong>Développer une architecture électronique</strong> robuste sur PCB (KiCad) pilotant les capteurs (présence, couleur) et un actionneur (moteur).</li>
                      <li><strong>Implémenter une communication inter-microcontrôleurs</strong> (I²C entre un ATmega328P et un ESP32) pour séparer la logique de contrôle temps réel de la connectivité réseau.</li>
                      <li><strong>Développer des firmwares embarqués</strong> structurés (machine à états) pour orchestrer l'ensemble du processus de tri de manière autonome.</li>
                      <li><strong>Fournir une solution électronique "clé en main"</strong>, testée et validée, prête à être interfacée avec le châssis du convoyeur et l'interface web.</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="2-spécifications--livrables" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FileText className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      2. Spécifications & Livrables
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Microcontrôleurs</strong> : Arduino Nano (ATmega328P) pour le contrôle, ESP32 pour la connectivité.</li>
                      <li><strong>Protocole de Communication</strong> : I²C (inter-MCU), Wi-Fi (vers le réseau).</li>
                      <li><strong>Technologies</strong> : KiCad 7, Arduino IDE, I²C.</li>
                    </ul>
                    <p><strong>Livrables de l'Équipe Électronique :</strong></p>
                    <div className="overflow-x-auto">
                      <table className="w-full text-sm">
                        <thead>
                          <tr className="bg-gray-50 dark:bg-gray-800">
                            <th className="px-4 py-2 text-left font-medium text-gray-900 dark:text-gray-100 border border-gray-200 dark:border-gray-700">Livrable</th>
                            <th className="px-4 py-2 text-left font-medium text-gray-900 dark:text-gray-100 border border-gray-200 dark:border-gray-700">Format</th>
                            <th className="px-4 py-2 text-left font-medium text-gray-900 dark:text-gray-100 border border-gray-200 dark:border-gray-700">Chemin d'accès</th>
                          </tr>
                        </thead>
                        <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
                          <tr><td className="px-4 py-2 border">Schéma Électronique Final</td><td className="px-4 py-2 border"><code>.kicad_sch</code></td><td className="px-4 py-2 border"><code>Documentation/test-final/elec/schematics/FinalTest_NanoEsp.kicad_sch</code></td></tr>
                          <tr><td className="px-4 py-2 border">Design PCB Final</td><td className="px-4 py-2 border"><code>.kicad_pcb</code></td><td className="px-4 py-2 border"><code>Documentation/test-final/elec/pcb/FinalTest_NanoEsp.kicad_pcb</code></td></tr>
                          <tr><td className="px-4 py-2 border">Firmware Contrôleur (Arduino Nano)</td><td className="px-4 py-2 border"><code>.ino</code></td><td className="px-4 py-2 border"><code>Documentation/test-final/elec/firmware/convoyeurArduino/</code></td></tr>
                          <tr><td className="px-4 py-2 border">Firmware Web API (ESP32)</td><td className="px-4 py-2 border"><code>.ino</code></td><td className="px-4 py-2 border"><code>Documentation/test-final/elec/firmware/convoyeurESP32/</code></td></tr>
                        </tbody>
                      </table>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="3-processus--workflow" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Layers className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      3. Processus & Workflow
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 mb-4">Notre projet a suivi une démarche d'ingénierie structurée :</p>
                    <ol className="list-decimal pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Phase de Conception</strong> : Conception itérative du circuit sur KiCad (voir Annexe B), passant de l'UART à l'I2C avec une solution de conversion de niveau logique optimisée pour garantir la fiabilité. Définition de l'architecture logicielle (machine à états sur Nano, API REST sur ESP32) et du protocole de communication.</li>
                      <li><strong>Développement Parallèle et Modulaire</strong> : Développement des firmwares Nano et ESP32. Prototypage et test unitaire de chaque sous-système (détection couleur, commande moteur, API web).</li>
                      <li><strong>Assemblage et Intégration</strong> : Assemblage des composants électroniques sur une carte de prototypage, et câblage des périphériques (capteurs, driver moteur).</li>
                      <li><strong>Calibration et Tests d'Intégration</strong> : Calibration fine des capteurs (couleur, présence) et ajustement des paramètres de la machine à états pour un fonctionnement optimal.</li>
                      <li><strong>Validation de bout en bout</strong> : Test du système électronique complet pour valider la chaîne, du signal du capteur de présence à la fourniture des données sur l'API web.</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="4-tâches--étapes" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      4. Tâches & Étapes
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="overflow-x-auto">
                      <table className="w-full text-sm">
                        <thead>
                          <tr className="bg-gray-50 dark:bg-gray-800">
                            <th className="px-4 py-2 text-left font-medium border">Étape</th>
                            <th className="px-4 py-2 text-left font-medium border">Responsable(s) Principal(aux)</th>
                            <th className="px-4 py-2 text-left font-medium border">Statut</th>
                          </tr>
                        </thead>
                        <tbody>
                          <tr><td className="px-4 py-2 border">Conception Schémas & PCB (KiCad)</td><td className="px-4 py-2 border">Livingstone GBOZO & Eunice ODJO</td><td className="px-4 py-2 border">✅ Terminé</td></tr>
                          <tr><td className="px-4 py-2 border">Développement Firmware (Nano & ESP32)</td><td className="px-4 py-2 border">Hugues HANTAN & Livingstone GBOZO</td><td className="px-4 py-2 border">✅ Terminé</td></tr>
                          <tr><td className="px-4 py-2 border">Assemblage Physique & Câblage</td><td className="px-4 py-2 border">Aretha FAGLA, Marielle AGBOSSOUNON & Hugues HANTAN</td><td className="px-4 py-2 border">✅ Terminé</td></tr>
                          <tr><td className="px-4 py-2 border">Tests & Calibration</td><td className="px-4 py-2 border">Hugues HANTAN, Eunice ODJO & Livingstone GBOZO</td><td className="px-4 py-2 border">✅ Terminé</td></tr>
                        </tbody>
                      </table>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="5-tests--validation" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      5. Tests & Validation
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 mb-4">Le système a été validé à travers une série de tests fonctionnels.</p>
                    <div className="overflow-x-auto">
                      <table className="w-full text-sm">
                        <thead>
                          <tr className="bg-gray-50 dark:bg-gray-800">
                            <th className="px-4 py-2 text-left font-medium border">Test</th>
                            <th className="px-4 py-2 text-left font-medium border">Objectif</th>
                            <th className="px-4 py-2 text-left font-medium border">Critère de Validation</th>
                            <th className="px-4 py-2 text-left font-medium border">Résultat</th>
                          </tr>
                        </thead>
                        <tbody>
                          <tr><td className="px-4 py-2 border">Détection de Présence</td><td className="px-4 py-2 border">Traiter les signaux des capteurs pour commander le moteur.</td><td className="px-4 py-2 border">Le moteur est commandé au démarrage quand le laser 1 est coupé et à l'arrêt quand le laser 2 est coupé.</td><td className="px-4 py-2 border">✅ Conforme</td></tr>
                          <tr><td className="px-4 py-2 border">Identification de Couleur</td><td className="px-4 py-2 border">Identifier correctement les 4 couleurs.</td><td className="px-4 py-2 border">Le système identifie la couleur du cube avec une précision > 95% après calibration.</td><td className="px-4 py-2 border">✅ Conforme</td></tr>
                          <tr><td className="px-4 py-2 border">Communication I²C</td><td className="px-4 py-2 border">Transmettre les données du Nano à l'ESP32 sans erreur.</td><td className="px-4 py-2 border">L'ESP32 reçoit et interprète correctement les données de couleur et les compteurs.</td><td className="px-4 py-2 border">✅ Conforme</td></tr>
                          <tr><td className="px-4 py-2 border">Validation de l'API Web</td><td className="px-4 py-2 border">Fournir les données de tri via une API HTTP.</td><td className="px-4 py-2 border">Le point d'API <code>/data</code> sur l'ESP32 renvoie un objet JSON valide avec les bonnes informations.</td><td className="px-4 py-2 border">✅ Conforme</td></tr>
                          <tr><td className="px-4 py-2 border">Cycle de Tri Complet</td><td className="px-4 py-2 border">Exécuter un cycle logique de tri de bout en bout de manière autonome.</td><td className="px-4 py-2 border">Un cube simulé à l'entrée déclenche le cycle de détection, transport, identification, et les données sont prêtes sur l'API.</td><td className="px-4 py-2 border">✅ Conforme</td></tr>
                        </tbody>
                      </table>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="6-fichiers-du-projet" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FolderArchive className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      6. Fichiers du Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">L'ensemble des fichiers sources de notre projet (Électronique, Firmware) est organisé et disponible dans les dossiers correspondants du dépôt.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="7-présentation-des-résultats" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BarChart2 className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      7. Présentation des Résultats
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200 mb-2">7.1 Conception Électronique (KiCad)</h3>
                      <p className="text-gray-700 dark:text-gray-300 mb-4">Le circuit a été conçu sur KiCad et intègre tous les composants sur un PCB unique pour une robustesse et une organisation optimales. L'évolution du design est détaillée en Annexe B.</p>
                      <div className="grid md:grid-cols-2 gap-4">
                        <figure><Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/Schema-V3-I2C-BSS.png" alt="Schéma KiCad Final (V3)" width={400} height={300} className="rounded-md border"/><figcaption className="text-sm italic text-center mt-2">Figure 1 : Schéma électrique final avec I2C et convertisseur de niveau.</figcaption></figure>
                        <figure><Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/PCB-V3-I2C-BSS.png" alt="PCB KiCad Final (V3)" width={400} height={300} className="rounded-md border"/><figcaption className="text-sm italic text-center mt-2">Figure 2 : Routage du PCB final.</figcaption></figure>
                      </div>
                      <figure className="mt-4"><Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/3D-V3-I2C-BSS.png" alt="Vue 3D du PCB Final" width={600} height={400} className="rounded-md border mx-auto"/><figcaption className="text-sm italic text-center mt-2">Figure 3 : PCB final avec I2C et convertisseur de niveau. Vue 3D</figcaption></figure>
                    </div>
                    <div>
                      <h3 className="font-semibold text-lg text-gray-800 dark:text-gray-200 mb-2">7.2 Prototype et Démonstration Fonctionnelle</h3>
                      <p className="text-gray-700 dark:text-gray-300 mb-4">Le système électronique a été assemblé pour validation. La vidéo et les images ci-dessous présentent un cycle complet de tri et les tests des fonctionnalités clés.</p>
                      <p className="text-gray-500 italic mb-4">[Placeholder pour la vidéo de démonstration finale]</p>
                      <div className="grid md:grid-cols-3 gap-4">
                        <figure><Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/prototype_convoyeur_photo.png" alt="Photo du prototype assemblé" width={300} height={200} className="rounded-md border"/><figcaption className="text-sm italic text-center mt-2">Figure 4 : Le système de contrôle assemblé et câblé.</figcaption></figure>
                        <figure className="flex items-center justify-center border rounded-md h-[200px] bg-gray-100 dark:bg-gray-800"><span className="text-sm text-gray-500 p-4 text-center">[Placeholder image/gif du cube coupant le faisceau]</span><figcaption className="text-sm italic text-center mt-2 sr-only">Figure 5 : Le Laser 1 détecte un cube et le firmware commande le moteur.</figcaption></figure>
                        <figure className="flex items-center justify-center border rounded-md h-[200px] bg-gray-100 dark:bg-gray-800"><span className="text-sm text-gray-500 p-4 text-center">[Placeholder image/gif du cube sous le capteur couleur]</span><figcaption className="text-sm italic text-center mt-2 sr-only">Figure 6 : Le système identifie un cube Rouge.</figcaption></figure>
                      </div>
                      <div className="grid md:grid-cols-3 gap-4 text-sm italic text-center mt-1">
                          <div></div>
                          <div>Figure 5 : Le Laser 1 détecte un cube et le firmware commande le moteur.</div>
                          <div>Figure 6 : Le système identifie un cube Rouge.</div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="8-ressources--références" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      8. Ressources & Références
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ul className="list-disc pl-5 space-y-2 text-gray-700 dark:text-gray-300">
                      <li><strong>Datasheets</strong> : ATmega328P, ESP32, TCS34725 (GY-33), L298N, BSS138.</li>
                      <li><strong>Logiciels</strong> : KiCad 7, Arduino IDE.</li>
                      <li><strong>Plateformes</strong> : GitHub.</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="9-annexes-techniques-détails" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <FileText className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      9. Annexes Techniques (Détails)
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <details className="group">
                      <summary className="flex items-center gap-2 font-medium cursor-pointer list-none">
                        <ArrowRight className="w-4 h-4 transition-transform group-open:rotate-90" />
                        Annexe A - Hardware et Schémas - L'Évolution de notre Conception Électronique
                      </summary>
                      <div className="mt-4 ml-6 space-y-4 text-gray-700 dark:text-gray-300">
                        <h4 className="font-semibold">1.1. Architecture Générale du Système Électronique</h4>
                        <p>L'architecture électronique du système de convoyeur est conçue pour assurer l'automatisation complète du processus de tri des déchets...</p>
                        <h4 className="font-semibold">1.2. Composants et Fonctionnement Détaillé</h4>
                        <h5 className="font-medium">1.2.1. Unité de Traitement Principale (Arduino Nano - ATmega328P)</h5>
                        <p>L'Arduino Nano, basé sur le microcontrôleur ATmega328P, constitue le cœur logique du système de tri...</p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/ArduinoNano.jpeg" alt="Arduino Nano" width={300} height={200} className="rounded-md border my-2"/>
                        <h5 className="font-medium">1.2.2. Module de Communication et Interface Web (ESP32 Dev Kit v1)</h5>
                        <p>Le module ESP32 Dev Kit v1 est spécifiquement intégré pour répondre à l'exigence d'une interface web...</p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/ESP32.jpeg" alt="ESP32" width={300} height={200} className="rounded-md border my-2"/>
                        <h5 className="font-medium">1.2.3. Convertisseur de Niveau Logique Bidirectionnel (Module BSS138)</h5>
                        <p>Le module convertisseur de niveau logique, basé sur le transistor MOSFET BSS138, est un composant essentiel...</p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/BSS138.jpeg" alt="Convertisseur BSS138" width={300} height={200} className="rounded-md border my-2"/>
                        <p><strong>Fonctionnement et Justification du Choix Optimal :</strong> Le principe de fonctionnement...</p>
                        <h5 className="font-medium">1.2.4. Capteur de Couleur (Module GY-33)</h5>
                        <p>Le module capteur de couleur GY-33 (TCS34725) identifie la couleur des déchets...</p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/GY33.jpeg" alt="Capteur GY-33" width={300} height={200} className="rounded-md border my-2"/>
                        <h5 className="font-medium">1.2.5. Capteurs de Présence (Modules Laser KY-008 et Photorésistances)</h5>
                        <p>Deux modules Laser KY-008 et des photorésistances détectent les déchets...</p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/KY008.jpeg" alt="Laser KY-008" width={300} height={200} className="rounded-md border my-2"/>
                        <h5 className="font-medium">1.2.6. Contrôle du Moteur du Convoyeur (Driver L298N)</h5>
                        <p>Le module L298N (pont en H) pilote le moteur DC...</p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/L298N.jpeg" alt="Driver L298N" width={300} height={200} className="rounded-md border my-2"/>
                        <h5 className="font-medium">1.2.7. Module d'Alimentation (Buck DC-DC - U3)</h5>
                        <p>Le régulateur Buck DC-DC convertit la tension variable de la batterie Lithium en un 5V stable...</p>
                        <h5 className="font-medium">1.2.8. Condensateurs de Découplage (C1, C2, C3, C4)</h5>
                        <p>Placés près des composants clés, ils filtrent le bruit et stabilisent les lignes d'alimentation...</p>
                      </div>
                    </details>
                    <details className="group">
                      <summary className="flex items-center gap-2 font-medium cursor-pointer list-none">
                        <ArrowRight className="w-4 h-4 transition-transform group-open:rotate-90" />
                        Annexe B - Évolution de la Conception du PCB et Schémas
                      </summary>
                      <div className="mt-4 ml-6 space-y-4 text-gray-700 dark:text-gray-300">
                        <p>Le développement de notre système électronique a suivi un processus itératif.</p>
                        <h5 className="font-medium">1.3.1. Version 1 : Schéma avec Liaison UART</h5>
                        <ul className="list-disc pl-5 text-sm">
                          <li><strong>Description</strong> : La première itération privilégiait une communication série UART...</li>
                          <li><strong>Schéma</strong> : <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/Schema-V1-UART.png" alt="Schéma V1 UART" width={400} height={300} className="rounded-md border my-2 inline-block"/></li>
                          <li><strong>PCB Associé</strong> : <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/PCB-V1-UART.png" alt="PCB V1 UART" width={400} height={300} className="rounded-md border my-2 inline-block"/></li>
                          <li><strong>Abandon</strong> : Complexité logicielle trop élevée...</li>
                        </ul>
                        <h5 className="font-medium">1.3.2. Version 2 : Schéma avec I2C et Résistances Pull-up Simples</h5>
                        <ul className="list-disc pl-5 text-sm">
                          <li><strong>Description</strong> : Migration vers le protocole I2C...</li>
                          <li><strong>Schéma</strong> : <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/Schema-V2-I2C-Pullup.png" alt="Schéma V2 I2C Pullup" width={400} height={300} className="rounded-md border my-2 inline-block"/></li>
                          <li><strong>Abandon</strong> : Analyse technique révélant une marge de sécurité trop faible...</li>
                        </ul>
                        <h5 className="font-medium">1.3.3. Version 3 (Finale) : Schéma avec I2C et Convertisseur de Niveau Dédié</h5>
                        <ul className="list-disc pl-5 text-sm">
                          <li><strong>Description</strong> : Solution finale et la plus robuste, utilisant un convertisseur...</li>
                          <li><strong>Schéma</strong> : <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/Schema-V3-I2C-BSS.png" alt="Schéma V3 I2C BSS" width={400} height={300} className="rounded-md border my-2 inline-block"/></li>
                          <li><strong>PCB Final</strong> : <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/elec/media/PCB-V3-I2C-BSS.png" alt="PCB V3 I2C BSS" width={400} height={300} className="rounded-md border my-2 inline-block"/></li>
                          <li><strong>Optimisation du PCB</strong> : Le design a été optimisé...</li>
                        </ul>
                      </div>
                    </details>
                    <details className="group">
                      <summary className="flex items-center gap-2 font-medium cursor-pointer list-none">
                        <ArrowRight className="w-4 h-4 transition-transform group-open:rotate-90" />
                        Annexe C - Gestion de l'Alimentation et des Câbles
                      </summary>
                      <div className="mt-4 ml-6 space-y-4 text-gray-700 dark:text-gray-300">
                        <h4 className="font-semibold">1.4. Gestion et Sécurité de l'Alimentation</h4>
                        <ul className="list-disc pl-5 text-sm">
                          <li><strong>Source</strong> : Bloc de batteries Lithium via un Jack DC (J5).</li>
                          <li><strong>Régulation</strong> : Module Buck DC-DC (U3) pour un 5V stable et précis.</li>
                          <li><strong>Stabilité</strong> : Condensateurs de découplage pour filtrer le bruit.</li>
                          <li><strong>Protection</strong> : Convertisseur de niveau logique pour protéger les broches...</li>
                        </ul>
                        <h4 className="font-semibold">1.5. Gestion des Câbles</h4>
                        <p>La conception du PCB facilite une gestion propre des câbles...</p>
                      </div>
                    </details>
                    <details className="group">
                      <summary className="flex items-center gap-2 font-medium cursor-pointer list-none">
                        <ArrowRight className="w-4 h-4 transition-transform group-open:rotate-90" />
                        Annexe D - Firmware (Code) - Le Cœur Logiciel
                      </summary>
                      <div className="mt-4 ml-6 space-y-4 text-gray-700 dark:text-gray-300">
                        <h4 className="font-semibold">2.1. Firmware Arduino Nano (Maître)</h4>
                        <p>Le code est structuré autour d'une <strong>machine à états finis</strong>...</p>
                        <ul className="list-disc pl-5 text-sm">
                          <li><strong>Architecture</strong> : Définition des broches, fonctions de contrôle...</li>
                          <li><strong>Machine à États (<code>ConveyorState</code>)</strong> : Gère le flux...</li>
                          <li><strong>Calibration</strong> : Une fonction <code>calibrateColorSensor()</code>...</li>
                        </ul>
                        <h4 className="font-semibold">2.2. Firmware ESP32 (Esclave et Web API)</h4>
                        <p>L'ESP32 agit comme un esclave I2C et un serveur web...</p>
                        <ul className="list-disc pl-5 text-sm">
                          <li><strong>Architecture</strong> : Paramètres Wi-Fi, adresse I2C...</li>
                          <li><strong>Communication I2C</strong> : La fonction <code>receiveEvent()</code>...</li>
                          <li><strong>Serveur Web et API</strong> : L'ESP32 initialise un serveur HTTP...</li>
                        </ul>
                      </div>
                    </details>
                    <details className="group">
                      <summary className="flex items-center gap-2 font-medium cursor-pointer list-none">
                        <ArrowRight className="w-4 h-4 transition-transform group-open:rotate-90" />
                        Annexe E - Instructions d'Utilisation et de Calibration
                      </summary>
                      <div className="mt-4 ml-6 space-y-4 text-gray-700 dark:text-gray-300">
                        <h4 className="font-semibold">3.1. Démarrage et Calibration Initiale</h4>
                        <ol className="list-decimal pl-5 text-sm">
                          <li><strong>Câblage</strong> : Vérifier le câblage selon le schéma final.</li>
                          <li><strong>Téléversement</strong> : Flasher le code sur le Nano, puis sur l'ESP32...</li>
                          <li><strong>Vérification IP</strong> : Noter l'adresse IP de l'ESP32...</li>
                          <li><strong>Calibration Couleur</strong> : Suivre les instructions sur le Moniteur Série...</li>
                        </ol>
                        <h4 className="font-semibold">3.2. Calibration des Paramètres de Fonctionnement (Variables à Ajuster)</h4>
                        <ul className="list-disc pl-5 text-sm">
                          <li><code>motorSpeedPWM</code> : Vitesse du moteur (0-255).</li>
                          <li><code>motorRunDuration_Start</code> : Durée du mouvement initial (en ms).</li>
                          <li><code>laserThreshold_1</code>, <code>laserThreshold_2</code> : Seuils de détection...</li>
                          <li><code>COLOR_MATCH_THRESHOLD</code> : Tolérance pour la reconnaissance de couleur.</li>
                        </ul>
                        <h4 className="font-semibold">3.3. Surveillance des Statistiques (Frontend Vercel)</h4>
                        <ol className="list-decimal pl-5 text-sm">
                          <li><strong>Accès</strong> : Ouvrir l'URL du frontend : <code>https://convoyeur-front-r5y5.vercel.app/</code></li>
                          <li><strong>Configuration</strong> : Mettre à jour l'URL de l'API...</li>
                          <li><strong>Observation</strong> : Les compteurs se mettent à jour en temps réel.</li>
                        </ol>
                      </div>
                    </details>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="10-conclusion" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      10. Conclusion
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      Ce projet final a mis en évidence notre capacité à mener un projet électronique complexe de bout en bout. De la conception itérative d'un PCB robuste à la programmation de firmwares embarqués communicants, nous avons transformé les exigences du cahier des charges en une solution matérielle et logicielle intégrée. Ce système de contrôle démontre notre maîtrise des capteurs, des actionneurs et des protocoles de communication, piliers fondamentaux de tout système robotique moderne et résilient.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
            </div>
          </div>
        </main>
      </div>
    </div>
  )
}