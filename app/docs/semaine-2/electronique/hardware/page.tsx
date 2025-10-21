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
  ArrowLeft, ArrowRight, Wrench, Cpu, Monitor, Layers, Puzzle, ExternalLink, Info, Users
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';
import { useMounted } from "@/hooks/use-mounted";


export default function HardwareGuideFRPage() {
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
            <section className="relative py-16 bg-gradient-to-br from-orange-50 via-white to-amber-50 dark:from-orange-950/20 dark:via-background dark:to-amber-950/20">
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
                    <span>Matériel Utilisé</span>
                  </div>

                  <div className="flex items-start gap-6 mb-8">
                    <div className="w-16 h-16 rounded-2xl bg-gradient-to-r from-orange-600 to-amber-600 flex items-center justify-center shadow-lg">
                      <Wrench className="w-8 h-8 text-white" />
                    </div>
                    <div className="flex-1">
                      <h1 className="text-4xl md:text-5xl font-bold bg-gradient-to-r from-orange-600 to-amber-600 bg-clip-text text-transparent mb-4">
                        Matériel Utilisé
                      </h1>
                      <p className="text-lg text-muted-foreground mb-6">
                        Liste détaillée du matériel requis pour construire et implémenter la Boîte Noire et la Station de Contrôle
                      </p>
                      <div className="flex flex-wrap gap-2">
                        <Badge variant="secondary" className="bg-orange-100 text-orange-700 dark:bg-orange-900/20 dark:text-orange-300">
                          Matériel
                        </Badge>
                        <Badge variant="outline">Électronique</Badge>
                        <Badge variant="outline">Composants</Badge>
                        <Badge variant="outline">Arduino</Badge>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-2/electronique/boite-noire">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Boîte Noire
                  </Button>
                </Link>
                <Link href="/docs/semaine-2/electronique/software-firmware">
                  <Button variant="ghost" size="sm">
                    Software & Firmware
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>

          {/* Content */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-5xl mx-auto">
              
              {/* Équipe et infos projet */}
              <AnimatedSection animation="fade-up">
                <Card className="border-l-4 border-l-blue-500 mb-8">
                  <CardHeader className="bg-gradient-to-r from-blue-50 to-transparent dark:from-blue-950/20">
                    <CardTitle className="flex items-center gap-2">
                      <Users className="w-5 h-5 text-blue-600" />
                      Équipe & Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <div className="grid md:grid-cols-2 gap-8">
                      <div>
                        <h3 className="text-lg font-semibold mb-4 text-gray-900 dark:text-gray-100">
                          Équipe IFRI Électronique
                        </h3>
                        <div className="space-y-3">
                          {[
                            "Aretha FAGLA",
                            "Hugues HANTAN", 
                            "Marielle AGBOSSOUNON",
                            "Eunice ODJO",
                            "Livingstone GBOZO"
                          ].map((name, index) => (
                            <div key={index} className="flex items-center gap-3">
                              <div className="w-2 h-2 bg-blue-500 rounded-full flex-shrink-0"></div>
                              <span className="text-sm text-gray-700 dark:text-gray-300">{name}</span>
                            </div>
                          ))}
                        </div>
                      </div>
                      
                      <div>
                        <h3 className="text-lg font-semibold mb-4 text-gray-900 dark:text-gray-100">
                          Informations du projet
                        </h3>
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
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Alerte d'information importante */}
              <AnimatedSection animation="fade-up" delay={25}>
                <Alert className="mb-8 border-orange-200 bg-orange-50 dark:border-orange-800 dark:bg-orange-950/20">
                  <Info className="h-4 w-4 text-orange-600" />
                  <AlertDescription className="text-orange-800 dark:text-orange-200">
                    <strong>Note importante :</strong> Cette liste présente le matériel essentiel pour le prototypage et l'implémentation finale du système. 
                    Les quantités exactes peuvent varier selon l'approche de montage choisie (breadboard vs PCB personnalisé).
                  </AlertDescription>
                </Alert>
              </AnimatedSection>
              
              <Separator className="my-8" />

              {/* SECTION 1 - Composants Communs */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-blue-500">
                  <CardHeader className="bg-gradient-to-r from-blue-50 to-transparent dark:from-blue-950/20">
                    <CardTitle id="composants-communs" className="flex items-center gap-2 scroll-mt-24 text-xl">
                      <Puzzle className="w-6 h-6 text-blue-600" />
                      1. Composants Clés Communs aux Deux Modules
                    </CardTitle>
                    <CardDescription>
                      Composants électroniques utilisés dans les deux modules du système
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-8 pt-6">
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Microcontrôleur ATmega328P</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2 space-y-3">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Le cœur de chaque module, responsable de l'exécution du firmware. Pour le prototypage, une carte Arduino Uno peut être utilisée.
                          </p>
                          <a href="https://ww1.microchip.com/downloads/en/DeviceAtmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf" target="_blank" rel="noopener noreferrer" 
                             className="text-sm text-blue-600 dark:text-blue-400 hover:underline inline-flex items-center gap-1">
                            <ExternalLink className="w-3 h-3" />
                            Fiche technique ATmega328P
                          </a>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/images/atmega328p_placeholder.webp" alt="Microcontrôleur ATmega328P" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Plaques d'Essai & Fils de Connexion</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Pour le prototypage initial et les interconnexions temporaires.
                          </p>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/Documentation/semaine-2/electronique/images/breadboard_jumpers.jpg" alt="Plaque d'essai et fils de connexion" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Adaptateur USB vers Série (FTDI ou Arduino Uno comme programmateur)</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2 space-y-3">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Essentiel pour le téléversement du code et la communication série (débogage) avec les ATmega autonomes.
                          </p>
                          <a href="https://ftdichip.com/wp-content/uploads/2020/08/DS_FT232R.pdf" target="_blank" rel="noopener noreferrer" 
                             className="text-sm text-blue-600 dark:text-blue-400 hover:underline inline-flex items-center gap-1">
                            <ExternalLink className="w-3 h-3" />
                            Exemple de fiche technique (FT232RL)
                          </a>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/Documentation/semaine-2/electronique/images/usb_to_serial_placeholder.jpeg" alt="Adaptateur USB vers Série" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 2 - Composants Boîte Noire */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Alert className="mb-6 border-gray-200 bg-gray-50 dark:border-gray-700 dark:bg-gray-950/20">
                  <Cpu className="h-4 w-4 text-gray-600" />
                  <AlertDescription className="text-gray-700 dark:text-gray-300">
                    La <strong>Boîte Noire</strong> contient le capteur principal MPU-6050 qui surveille en permanence l'orientation et les mouvements du drone.
                  </AlertDescription>
                </Alert>
                
                <Card className="border-l-4 border-l-gray-700">
                  <CardHeader className="bg-gradient-to-r from-gray-50 to-transparent dark:from-gray-950/20">
                    <CardTitle id="composants-boite-noire" className="flex items-center gap-2 scroll-mt-24 text-xl">
                      <Cpu className="w-6 h-6 text-gray-700" />
                      2. Composants Spécifiques à la Boîte Noire
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Module Capteur MPU-6050 (Gyroscope + Accéléromètre)</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2 space-y-3">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Le capteur 6 DOF (Degrés de Liberté) qui fournit les données d'orientation et d'accélération pour la détection de vol et de crash.
                          </p>
                          <a href="https://www.cdiweb.com/datasheets/invensense/MPU-6050_DataSheet_V3.4.pdf" target="_blank" rel="noopener noreferrer" 
                             className="text-sm text-blue-600 dark:text-blue-400 hover:underline inline-flex items-center gap-1">
                            <ExternalLink className="w-3 h-3" />
                            Fiche technique de la puce MPU-6050
                          </a>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/Documentation/semaine-2/electronique/images/mpu6050_placeholder.jpg" alt="Module MPU-6050" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* SECTION 3 - Composants Station de Contrôle */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Alert className="mb-6 mt-12 border-green-200 bg-green-50 dark:border-green-800 dark:bg-green-950/20">
                  <Monitor className="h-4 w-4 text-green-600" />
                  <AlertDescription className="text-green-800 dark:text-green-200">
                    La <strong>Station de Contrôle</strong> affiche en temps réel les données de vol et permet de visualiser l'état du système via un écran LCD.
                  </AlertDescription>
                </Alert>
                
                <Card className="border-l-4 border-l-green-500">
                  <CardHeader className="bg-gradient-to-r from-green-50 to-transparent dark:from-green-950/20">
                    <CardTitle id="composants-station" className="flex items-center gap-2 scroll-mt-24 text-xl">
                      <Monitor className="w-6 h-6 text-green-600" />
                      3. Composants Spécifiques à la Station de Contrôle
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Écran LCD 16x2 ou 20x4 avec Module I2C</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2 space-y-3">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            L'interface visuelle pour l'affichage en temps réel des données de vol et du statut de crash. L'adaptateur I2C simplifie grandement le câblage.
                          </p>
                          <div className="flex flex-col gap-2">
                            <a href="https://www.sparkfun.com/datasheets/LCD/HD44780.pdf" target="_blank" rel="noopener noreferrer" 
                               className="text-sm text-blue-600 dark:text-blue-400 hover:underline inline-flex items-center gap-1">
                              <ExternalLink className="w-3 h-3" />
                              Exemple de fiche technique (HD44780)
                            </a>
                            <a href="https://www.ti.com/lit/ds/symlink/pcf8574.pdf" target="_blank" rel="noopener noreferrer" 
                               className="text-sm text-blue-600 dark:text-blue-400 hover:underline inline-flex items-center gap-1">
                              <ExternalLink className="w-3 h-3" />
                              Exemple de fiche technique I2C (PCF8574)
                            </a>
                          </div>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/Documentation/semaine-2/electronique/images/lcd_i2c_placeholder.jpeg" alt="Module LCD I2C" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 4 - Composants PCB */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-orange-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-orange-50 to-transparent dark:from-orange-950/20">
                    <CardTitle id="composants-pcb" className="flex items-center gap-2 scroll-mt-24 text-xl">
                      <Layers className="w-6 h-6 text-orange-600" />
                      4. Composants pour la Conception de PCB (si applicable)
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-8 pt-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Quartz 16 MHz et Condensateurs 22 pF (si ATmega328P autonome)</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Nécessaires pour un fonctionnement stable de l'horloge de l'ATmega328P lorsqu'il est utilisé seul sur un PCB personnalisé.
                          </p>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/Documentation/semaine-2/electronique/images/quartz_capacitors.jpeg" alt="Quartz et Condensateurs" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Résistances de Pull-up (4.7kΩ, pour I2C si non intégrées)</h4>
                      <div className="grid md:grid-cols-3 gap-6">
                        <div className="md:col-span-2">
                          <p className="text-sm text-gray-600 dark:text-gray-400">
                            Assurent la tension de repos correcte sur les lignes SDA et SCL du bus I2C.
                          </p>
                        </div>
                        <div className="md:col-span-1">
                          <Image src="/Documentation/semaine-2/electronique/images/resistors_pullup.jpeg" alt="Résistances de Pull-up" width={200} height={150} className="rounded-md border mx-auto" />
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation finale */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="mt-12 bg-gradient-to-r from-gray-50 to-white dark:from-gray-900/50 dark:to-background border border-gray-200 dark:border-gray-700">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between">
                      <Link href="/docs/semaine-2/electronique/boite-noire">
                        <Button variant="outline" className="gap-2">
                          <ArrowLeft className="w-4 h-4" />
                          Boîte Noire & Station de Contrôle
                        </Button>
                      </Link>
                      
                      <div className="text-center">
                        <div className="text-sm text-muted-foreground mb-1">Section suivante</div>
                        <div className="text-sm font-medium">Software & Firmware</div>
                      </div>
                      
                      <Link href="/docs/semaine-2/electronique/software-firmware">
                        <Button className="gap-2 bg-orange-600 hover:bg-orange-700">
                          Software & Firmware
                          <ArrowRight className="w-4 h-4" />
                        </Button>
                      </Link>
                    </div>
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