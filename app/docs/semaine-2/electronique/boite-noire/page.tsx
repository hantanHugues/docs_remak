"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import { Alert, AlertDescription } from "@/components/ui/alert";
import {
  ArrowLeft, ArrowRight, Wrench, DownloadCloud, Cpu, Monitor, Network, Power,
  Settings, Check, AlertTriangle, FileText, Zap, Plug
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

export default function BlackBoxControlStationPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-16 bg-gradient-to-br from-blue-50 via-white to-indigo-50 dark:from-blue-950/20 dark:via-background dark:to-indigo-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-6">
                    <Link href="/docs/semaine-2" className="hover:text-foreground transition-colors">
                      Semaine 2
                    </Link>
                    <span>/</span>
                    <Link href="/docs/semaine-2/electronique" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Boîte Noire & Station de Contrôle</span>
                  </div>

                  <div className="flex items-start gap-6 mb-8">
                    <div className="w-16 h-16 rounded-2xl bg-gradient-to-r from-blue-600 to-indigo-600 flex items-center justify-center shadow-lg">
                      <Wrench className="w-8 h-8 text-white" />
                    </div>
                    <div className="flex-1">
                      <h1 className="text-4xl md:text-5xl font-bold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent mb-4">
                        Guide d'Assemblage et Configuration
                      </h1>
                      <p className="text-xl text-muted-foreground mb-6">
                        Instructions détaillées pour l'assemblage et la configuration des modules Boîte Noire et Station de Contrôle
                      </p>
                      <div className="flex flex-wrap gap-2">
                        <Badge className="bg-blue-100 text-blue-700 border-blue-200 dark:bg-blue-900/30 dark:text-blue-300 dark:border-blue-800">
                          <Cpu className="w-3 h-3 mr-1" />
                          Arduino
                        </Badge>
                        <Badge className="bg-green-100 text-green-700 border-green-200 dark:bg-green-900/30 dark:text-green-300 dark:border-green-800">
                          <Zap className="w-3 h-3 mr-1" />
                          MPU-6050
                        </Badge>
                        <Badge className="bg-purple-100 text-purple-700 border-purple-200 dark:bg-purple-900/30 dark:text-purple-300 dark:border-purple-800">
                          <Monitor className="w-3 h-3 mr-1" />
                          LCD I2C
                        </Badge>
                        <Badge variant="outline">KiCad</Badge>
                        <Badge variant="outline">PCB Design</Badge>
                      </div>
                    </div>
                  </div>

                  <Alert>
                    <AlertTriangle className="h-4 w-4" />
                    <AlertDescription>
                      <strong>Prérequis :</strong> Connaissances de base en électronique, programmation Arduino et utilisation de l'IDE Arduino.
                    </AlertDescription>
                  </Alert>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-2/electronique">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Électronique
                  </Button>
                </Link>
                <div className="flex items-center gap-2 text-sm text-muted-foreground">
                  <FileText className="w-4 h-4" />
                  <span>Guide d'assemblage</span>
                </div>
                <Link href="#">
                  <Button variant="ghost" size="sm">
                    Suivant
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>

          {/* Contenu */}
          <div className="container mx-auto px-4 py-12">
            <div className="max-w-4xl mx-auto space-y-12">
              
              {/* Section 1: Configuration Environnement */}
              <AnimatedSection animation="fade-up">
                <Card className="border-l-4 border-l-blue-500">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="w-10 h-10 rounded-lg bg-blue-100 dark:bg-blue-900/30 flex items-center justify-center">
                        <DownloadCloud className="w-5 h-5 text-blue-600 dark:text-blue-400" />
                      </div>
                      <div>
                        <CardTitle className="text-xl">1. Configuration de l'Environnement de Développement</CardTitle>
                        <CardDescription>Préparation de l'IDE Arduino et installation des bibliothèques</CardDescription>
                      </div>
                    </div>
                  </CardHeader>
                  <CardContent className="space-y-6">
                    <div className="prose prose-gray dark:prose-invert max-w-none">
                      <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                        <Settings className="w-5 h-5 text-blue-600" />
                        Installation de l'IDE Arduino
                      </h4>
                      <div className="bg-blue-50 dark:bg-blue-900/20 p-4 rounded-lg mb-4">
                        <p className="font-medium mb-2">Étapes d'installation :</p>
                        <ol className="space-y-2">
                          <li className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Télécharger l'IDE Arduino depuis <a href="https://www.arduino.cc/en/software" target="_blank" rel="noopener noreferrer" className="text-blue-600 hover:underline">arduino.cc/en/software</a></span>
                          </li>
                          <li className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Installer l'IDE en suivant les instructions de votre système d'exploitation</span>
                          </li>
                          <li className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Lancer l'IDE et vérifier le bon fonctionnement</span>
                          </li>
                        </ol>
                      </div>

                      <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                        <FileText className="w-5 h-5 text-purple-600" />
                        Bibliothèques Requises
                      </h4>
                      <div className="grid md:grid-cols-3 gap-4">
                        <Card className="p-4">
                          <h5 className="font-semibold text-green-700 dark:text-green-400 mb-2">Adafruit MPU6050</h5>
                          <p className="text-sm text-muted-foreground">Gestion du capteur gyroscope/accéléromètre</p>
                        </Card>
                        <Card className="p-4">
                          <h5 className="font-semibold text-blue-700 dark:text-blue-400 mb-2">Adafruit Unified Sensor</h5>
                          <p className="text-sm text-muted-foreground">Dépendance pour MPU6050</p>
                        </Card>
                        <Card className="p-4">
                          <h5 className="font-semibold text-purple-700 dark:text-purple-400 mb-2">LiquidCrystal I2C</h5>
                          <p className="text-sm text-muted-foreground">Contrôle de l'écran LCD</p>
                        </Card>
                      </div>

                      <Alert className="mt-4">
                        <FileText className="h-4 w-4" />
                        <AlertDescription>
                          <strong>Installation :</strong> Aller dans <code>Croquis → Inclure une bibliothèque → Gérer les bibliothèques...</code> et rechercher chaque bibliothèque.
                        </AlertDescription>
                      </Alert>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator className="my-8" />

              {/* Section 2: Module Boîte Noire */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-green-500">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="w-10 h-10 rounded-lg bg-green-100 dark:bg-green-900/30 flex items-center justify-center">
                        <Cpu className="w-5 h-5 text-green-600 dark:text-green-400" />
                      </div>
                      <div>
                        <CardTitle className="text-xl">2. Module Boîte Noire (BlackBox)</CardTitle>
                        <CardDescription>Assemblage du microcontrôleur ATmega328P avec le capteur MPU-6050</CardDescription>
                      </div>
                    </div>
                  </CardHeader>
                  <CardContent className="space-y-8">
                    <div className="prose prose-gray dark:prose-invert max-w-none">
                      <div className="bg-green-50 dark:bg-green-900/20 p-6 rounded-lg">
                        <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                          <Plug className="w-5 h-5 text-green-600" />
                          Schéma de Câblage - Prototypage Breadboard
                        </h4>
                        <div className="grid md:grid-cols-2 gap-6">
                          <div>
                            <h5 className="font-semibold mb-3">ATmega328P (Arduino Uno)</h5>
                            <p className="text-sm text-muted-foreground">Microcontrôleur principal du module</p>
                          </div>
                          <div>
                            <h5 className="font-semibold mb-3">Connexions MPU-6050</h5>
                            <div className="space-y-2 text-sm">
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">VCC</code>
                                <span>→</span>
                                <code className="bg-red-100 dark:bg-red-900/30 px-2 py-1 rounded">5V</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">GND</code>
                                <span>→</span>
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">GND</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">SDA</code>
                                <span>→</span>
                                <code className="bg-blue-100 dark:bg-blue-900/30 px-2 py-1 rounded">A4</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">SCL</code>
                                <span>→</span>
                                <code className="bg-blue-100 dark:bg-blue-900/30 px-2 py-1 rounded">A5</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">INT</code>
                                <span>→</span>
                                <code className="bg-purple-100 dark:bg-purple-900/30 px-2 py-1 rounded">D2</code>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div className="grid gap-6 mt-8">
                        <div>
                          <h4 className="text-lg font-semibold mb-4">Schéma KiCad de la Boîte Noire</h4>
                          <div className="bg-white dark:bg-gray-900 p-4 rounded-lg border">
                            <Image 
                              src="/Documentation/semaine-2/electronique/images/cube_schema.png" 
                              alt="Schéma KiCad de la Boîte Noire" 
                              width={700} 
                              height={500} 
                              className="rounded-md w-full"
                            />
                            <div className="mt-3 flex items-center gap-2 text-sm text-muted-foreground">
                              <FileText className="w-4 h-4" />
                              <span>Fichier source : </span>
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-xs">
                                hardware/cube_pcb/cube_pcb.kicad_sch
                              </code>
                            </div>
                          </div>
                        </div>

                        <div>
                          <h4 className="text-lg font-semibold mb-4">Visualisation 3D du PCB</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="bg-white dark:bg-gray-900 p-4 rounded-lg border">
                              <Image 
                                src="/Documentation/semaine-2/electronique/images/cube_pcb_PCB_3DViewer1.png" 
                                alt="Vue 3D PCB Boîte Noire - Face 1" 
                                width={350} 
                                height={300} 
                                className="rounded-md w-full"
                              />
                            </div>
                            <div className="bg-white dark:bg-gray-900 p-4 rounded-lg border">
                              <Image 
                                src="/Documentation/semaine-2/electronique/images/cube_pcb_PCB_3DViewer2.png" 
                                alt="Vue 3D PCB Boîte Noire - Face 2" 
                                width={350} 
                                height={300} 
                                className="rounded-md w-full"
                              />
                            </div>
                          </div>
                          <div className="mt-3 flex items-center gap-2 text-sm text-muted-foreground">
                            <FileText className="w-4 h-4" />
                            <span>Fichier PCB : </span>
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-xs">
                              hardware/cube_pcb/cube_pcb.kicad_pcb
                            </code>
                          </div>
                        </div>


                      </div>

                      <div className="bg-blue-50 dark:bg-blue-900/20 p-6 rounded-lg mt-8">
                        <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                          <DownloadCloud className="w-5 h-5 text-blue-600" />
                          Téléversement du Firmware
                        </h4>
                        <div className="space-y-3">
                          <div className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Ouvrir le fichier <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">cube_firmware.ino</code> dans l'IDE Arduino</span>
                          </div>
                          <div className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Sélectionner la carte "Arduino Uno" et le port COM/USB approprié</span>
                          </div>
                          <div className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Cliquer sur "Téléverser" (flèche droite) pour compiler et charger le firmware</span>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator className="my-8" />

              {/* Section 3: Module Station de Contrôle */}
              <AnimatedSection animation="fade-up" delay={200}>
                <Card className="border-l-4 border-l-purple-500">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="w-10 h-10 rounded-lg bg-purple-100 dark:bg-purple-900/30 flex items-center justify-center">
                        <Monitor className="w-5 h-5 text-purple-600 dark:text-purple-400" />
                      </div>
                      <div>
                        <CardTitle className="text-xl">3. Module Station de Contrôle</CardTitle>
                        <CardDescription>Assemblage du microcontrôleur ATmega328P avec l'écran LCD I2C</CardDescription>
                      </div>
                    </div>
                  </CardHeader>
                  <CardContent className="space-y-8">
                    <div className="prose prose-gray dark:prose-invert max-w-none">
                      <div className="bg-purple-50 dark:bg-purple-900/20 p-6 rounded-lg">
                        <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                          <Plug className="w-5 h-5 text-purple-600" />
                          Schéma de Câblage - Prototypage Breadboard
                        </h4>
                        <div className="grid md:grid-cols-2 gap-6">
                          <div>
                            <h5 className="font-semibold mb-3">ATmega328P (Arduino Uno)</h5>
                            <p className="text-sm text-muted-foreground">Microcontrôleur principal du module</p>
                          </div>
                          <div>
                            <h5 className="font-semibold mb-3">Connexions LCD I2C</h5>
                            <div className="space-y-2 text-sm">
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">VCC</code>
                                <span>→</span>
                                <code className="bg-red-100 dark:bg-red-900/30 px-2 py-1 rounded">5V</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">GND</code>
                                <span>→</span>
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">GND</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">SDA</code>
                                <span>→</span>
                                <code className="bg-blue-100 dark:bg-blue-900/30 px-2 py-1 rounded">A4</code>
                              </div>
                              <div className="flex justify-between">
                                <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">SCL</code>
                                <span>→</span>
                                <code className="bg-blue-100 dark:bg-blue-900/30 px-2 py-1 rounded">A5</code>
                              </div>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div className="grid gap-6 mt-8">
                        <div>
                          <h4 className="text-lg font-semibold mb-4">Schéma KiCad de la Station de Contrôle</h4>
                          <div className="bg-white dark:bg-gray-900 p-4 rounded-lg border">
                            <Image 
                              src="/Documentation/semaine-2/electronique/images/station_schema.png" 
                              alt="Schéma KiCad de la Station de Contrôle" 
                              width={700} 
                              height={500} 
                              className="rounded-md w-full"
                            />
                            <div className="mt-3 flex items-center gap-2 text-sm text-muted-foreground">
                              <FileText className="w-4 h-4" />
                              <span>Fichier source : </span>
                              <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-xs">
                                hardware/station_pcb/station_pcb.kicad_sch
                              </code>
                            </div>
                          </div>
                        </div>

                        <div>
                          <h4 className="text-lg font-semibold mb-4">Visualisation 3D du PCB</h4>
                          <div className="grid md:grid-cols-2 gap-4">
                            <div className="bg-white dark:bg-gray-900 p-4 rounded-lg border">
                              <Image 
                                src="/Documentation/semaine-2/electronique/images/station_pcb_PCB_3Dviewer1.png" 
                                alt="Vue 3D PCB Station de Contrôle - Face 1" 
                                width={350} 
                                height={300} 
                                className="rounded-md w-full"
                              />
                            </div>
                            <div className="bg-white dark:bg-gray-900 p-4 rounded-lg border">
                              <Image 
                                src="/Documentation/semaine-2/electronique/images/station_pcb_PCB_3Dviewer2.png" 
                                alt="Vue 3D PCB Station de Contrôle - Face 2" 
                                width={350} 
                                height={300} 
                                className="rounded-md w-full"
                              />
                            </div>
                          </div>
                          <div className="mt-3 flex items-center gap-2 text-sm text-muted-foreground">
                            <FileText className="w-4 h-4" />
                            <span>Fichier PCB : </span>
                            <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded text-xs">
                              hardware/station_pcb/station_pcb.kicad_pcb
                            </code>
                          </div>
                        </div>


                      </div>

                      <div className="bg-purple-50 dark:bg-purple-900/20 p-6 rounded-lg mt-8">
                        <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                          <DownloadCloud className="w-5 h-5 text-purple-600" />
                          Téléversement du Firmware
                        </h4>
                        <div className="space-y-3">
                          <div className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Ouvrir le fichier <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">station_firmware.ino</code> dans l'IDE Arduino</span>
                          </div>
                          <div className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Sélectionner la bonne carte et le port COM/USB approprié</span>
                          </div>
                          <div className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Cliquer sur "Téléverser" pour compiler et charger le firmware</span>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator className="my-8" />

              {/* Section 4: Configuration I2C */}
              <AnimatedSection animation="fade-up" delay={300}>
                <Card className="border-l-4 border-l-orange-500">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="w-10 h-10 rounded-lg bg-orange-100 dark:bg-orange-900/30 flex items-center justify-center">
                        <Network className="w-5 h-5 text-orange-600 dark:text-orange-400" />
                      </div>
                      <div>
                        <CardTitle className="text-xl">4. Configuration de l'Adresse I2C</CardTitle>
                        <CardDescription>Paramétrage de la communication entre les modules</CardDescription>
                      </div>
                    </div>
                  </CardHeader>
                  <CardContent className="space-y-6">
                    <div className="prose prose-gray dark:prose-invert max-w-none">
                      <Alert className="mb-6">
                        <Network className="h-4 w-4" />
                        <AlertDescription>
                          <strong>Important :</strong> L'adresse I2C doit être identique dans les deux modules pour assurer la communication.
                        </AlertDescription>
                      </Alert>

                      <div className="bg-orange-50 dark:bg-orange-900/20 p-6 rounded-lg">
                        <h4 className="text-lg font-semibold mb-4">Configuration de l'adresse</h4>
                        <ul className="space-y-2">
                          <li className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>Vérifier que <code className="bg-gray-100 dark:bg-gray-800 px-2 py-1 rounded">STATION_ADDRESS</code> est identique dans les deux firmwares</span>
                          </li>
                          <li className="flex items-start gap-2">
                            <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                            <span>L'adresse par défaut est définie sur <code className="bg-orange-100 dark:bg-orange-900/30 px-2 py-1 rounded">8</code></span>
                          </li>
                        </ul>
                      </div>

                      <div className="bg-gray-900 rounded-lg p-6 my-6 overflow-x-auto">
                        <div className="flex items-center justify-between mb-4">
                          <h5 className="text-white font-semibold">Configuration I2C</h5>
                          <span className="text-xs text-gray-400 bg-gray-800 px-2 py-1 rounded">cpp</span>
                        </div>
                        <pre className="text-sm text-gray-100">
                          <code>{`// Dans cube_firmware.ino et station_firmware.ino
const int STATION_ADDRESS = 8;

// Configuration I2C pour Arduino Uno/ATmega328P
// SDA: A4 (Pin analogique 4)
// SCL: A5 (Pin analogique 5)`}</code>
                        </pre>
                      </div>

                      <Alert>
                        <Settings className="h-4 w-4" />
                        <AlertDescription>
                          <strong>Note technique :</strong> Les broches SDA (A4) et SCL (A5) sont standard sur Arduino Uno. Pour ATmega328P autonome, s'assurer que ces broches sont correctement connectées.
                        </AlertDescription>
                      </Alert>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <Separator className="my-8" />

              {/* Section 5: Alimentation */}
              <AnimatedSection animation="fade-up" delay={400}>
                <Card className="border-l-4 border-l-yellow-500">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="w-10 h-10 rounded-lg bg-yellow-100 dark:bg-yellow-900/30 flex items-center justify-center">
                        <Power className="w-5 h-5 text-yellow-600 dark:text-yellow-400" />
                      </div>
                      <div>
                        <CardTitle className="text-xl">5. Alimentation des Modules</CardTitle>
                        <CardDescription>Recommandations pour l'alimentation électrique</CardDescription>
                      </div>
                    </div>
                  </CardHeader>
                  <CardContent className="space-y-6">
                    <div className="prose prose-gray dark:prose-invert max-w-none">
                      <div className="grid md:grid-cols-2 gap-6">
                        <Card className="p-6 bg-yellow-50 dark:bg-yellow-900/20">
                          <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                            <Zap className="w-5 h-5 text-yellow-600" />
                            Spécifications
                          </h4>
                          <div className="space-y-3">
                            <div className="flex justify-between items-center">
                              <span className="font-medium">Tension :</span>
                              <Badge variant="outline" className="bg-red-100 text-red-700 border-red-200">5V DC</Badge>
                            </div>
                            <div className="flex justify-between items-center">
                              <span className="font-medium">Courant :</span>
                              <Badge variant="outline">~200mA par module</Badge>
                            </div>
                            <div className="flex justify-between items-center">
                              <span className="font-medium">Stabilité :</span>
                              <Badge variant="outline" className="bg-green-100 text-green-700 border-green-200">±5%</Badge>
                            </div>
                          </div>
                        </Card>

                        <Card className="p-6 bg-red-50 dark:bg-red-900/20">
                          <h4 className="flex items-center gap-2 text-lg font-semibold mb-4">
                            <AlertTriangle className="w-5 h-5 text-red-600" />
                            Considérations Importantes
                          </h4>
                          <div className="space-y-3">
                            <div className="flex items-start gap-2">
                              <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                              <span className="text-sm">Masses (GND) interconnectées</span>
                            </div>
                            <div className="flex items-start gap-2">
                              <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                              <span className="text-sm">Alimentation stable et filtrée</span>
                            </div>
                            <div className="flex items-start gap-2">
                              <Check className="w-4 h-4 text-green-600 mt-0.5 flex-shrink-0" />
                              <span className="text-sm">Protection contre les inversions</span>
                            </div>
                          </div>
                        </Card>
                      </div>

                      <Alert className="mt-6">
                        <Power className="h-4 w-4" />
                        <AlertDescription>
                          <strong>Sécurité :</strong> Toujours vérifier la polarité et la tension avant de connecter l'alimentation. Une mauvaise connexion peut endommager les composants.
                        </AlertDescription>
                      </Alert>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation finale */}
              <div className="flex items-center justify-between pt-12 border-t border-border/50">
                <Link href="/docs/semaine-2/electronique">
                  <Button variant="outline" size="lg">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Retour à Électronique
                  </Button>
                </Link>
                <div className="text-center">
                  <p className="text-sm text-muted-foreground">Guide d'assemblage terminé</p>
                  <p className="text-xs text-muted-foreground mt-1">Prêt pour les tests de fonctionnement</p>
                </div>
                <Link href="#">
                  <Button size="lg">
                    Tests & Validation
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