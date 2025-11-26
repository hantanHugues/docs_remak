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
  ArrowLeft, ArrowRight, Code, Cpu, Monitor, FileCode, Workflow, Database, RefreshCcw, BookOpen, AlertTriangle
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';
import { useMounted } from "@/hooks/use-mounted";


export default function FirmwareGuideFRPage() {
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
            <section className="relative py-12 bg-gradient-to-br from-purple-50 via-white to-indigo-50 dark:from-purple-950/20 dark:via-background dark:to-indigo-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Logiciel (Firmware)</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Code className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Logiciel (Firmware) - Boîte Noire & Station de Contrôle</h1>
                      <p className="text-muted-foreground">Structure et modularité du code embarqué pour les microcontrôleurs ATmega328P.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Firmware</Badge>
                    <Badge variant="outline">Arduino</Badge>
                    <Badge variant="outline">Logiciel</Badge>
                    <Badge variant="outline">I2C</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <PageNavigation />

          {/* Contenu */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-5xl mx-auto">
              
              <AnimatedSection animation="fade-up">
                <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-8 mb-12 text-center">
                  <p className="text-gray-700 dark:text-gray-300 leading-relaxed mb-4">
                    Le cœur de ce projet réside dans le firmware embarqué, développé avec l'IDE Arduino pour les microcontrôleurs ATmega328P. Le code est structuré pour maximiser la clarté et la modularité.
                  </p>
                  <Button asChild>
                    <Link href="/Documentation/semaine-2/electronique/firmware/">
                      <FileCode className="w-4 h-4 mr-2" />
                      Accéder au code source complet
                    </Link>
                  </Button>
                </div>
              </AnimatedSection>
              
              <Separator className="mb-12" />

              {/* SECTION 1 - Firmware de la Boîte Noire */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-gray-700 mt-12">
                  <CardHeader className="bg-gradient-to-r from-gray-50 to-transparent dark:from-gray-950/20">
                    <CardTitle id={slugify("1. Firmware de la Boîte Noire")} className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <Cpu className="w-6 h-6 text-gray-700" />
                      1. Firmware de la Boîte Noire (<code>cube_firmware.ino</code>)
                    </CardTitle>
                    <CardDescription>
                      <Link href="/Documentation/semaine-2/electronique/firmware/cube_firmware/cube_firmware.ino" className="text-sm text-blue-600 dark:text-blue-400 hover:underline">
                        Chemin : <code>firmware/cube_firmware/cube_firmware.ino</code>
                      </Link>
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">
                    <div className="bg-gray-50 dark:bg-gray-900 border border-gray-200 dark:border-gray-700 p-4 rounded-lg">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Rôle Principal</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        Acquisition des données du MPU-6050, détection de crash, sauvegarde des données critiques en EEPROM et transmission I2C vers la Station de Contrôle.
                      </p>
                    </div>

                    <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                      Fonctions Clés et Leurs Rôles
                    </h3>
                    
                    <div className="space-y-4">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>setup()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Initialise le MPU-6050 et le bus I2C. Gère la logique de réinitialisation autonome du système après un crash (détecté par un cycle d'alimentation) et configure l'interruption "Data Ready" du MPU-6050.
                        </p>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>loop()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Le cœur du programme. Lit en continu les données du MPU-6050, applique la logique de détection de crash, gère l'enregistrement dans le tampon circulaire de l'EEPROM et envoie les données à la Station de Contrôle via I2C.
                        </p>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Structure <code>FlightData</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
                          Définit la structure des données (Roulis, Tangage, Accélération Z, Statut) échangées entre la Boîte Noire et la Station de Contrôle.
                        </p>
                        <Image src="/2025-Team-IFRI-Docs/Documentation/semaine-2/electronique/images/flight_data_structure.png" alt="Diagramme de la structure FlightData" width={600} height={200} className="rounded-md border mx-auto" />
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>handleSerialRecovery()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Permet de récupérer les données enregistrées dans l'EEPROM via le port série après un crash, lors de l'envoi d'une commande spécifique.
                        </p>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>handleMPUInterrupt()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Routine de service d'interruption pour le MPU-6050 (définit un drapeau pour la lecture des données).
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 2 - Firmware de la Station de Contrôle */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-green-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-green-50 to-transparent dark:from-green-950/20">
                    <CardTitle id={slugify("2. Firmware de la Station de Contrôle")} className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <Monitor className="w-6 h-6 text-green-600" />
                      2. Firmware de la Station de Contrôle (<code>station_firmware.ino</code>)
                    </CardTitle>
                    <CardDescription>
                      <Link href="/Documentation/semaine-2/electronique/firmware/station_firmware/station_firmware.ino" className="text-sm text-blue-600 dark:text-blue-400 hover:underline">
                        Chemin : <code>firmware/station_firmware/station_firmware.ino</code>
                      </Link>
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">
                    <div className="bg-gray-50 dark:bg-gray-900 border border-gray-200 dark:border-gray-700 p-4 rounded-lg">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Rôle Principal</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        Réception des données I2C provenant de la Boîte Noire et leur affichage sur l'écran LCD.
                      </p>
                    </div>
                    
                    <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 border-b border-gray-200 dark:border-gray-700 pb-2">
                      Fonctions Clés et Leurs Rôles
                    </h3>

                    <div className="space-y-4">
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>setup()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Initialise le microcontrôleur en tant qu'esclave I2C et configure l'écran LCD.
                        </p>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>loop()</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Gère la mise à jour de l'affichage LCD en fonction des données reçues et du statut de crash ("Normal" ou "CRASH !!!").
                        </p>
                      </div>

                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2"><code>receiveEvent(int numBytes)</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Fonction de rappel I2C. Elle est automatiquement appelée lorsque des données sont reçues de la Boîte Noire et les lit dans la structure <code>FlightData</code>.
                        </p>
                      </div>
                      
                      <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                        <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Structure <code>FlightData</code></h4>
                        <p className="text-sm text-gray-600 dark:text-gray-400">
                          Utilise la même structure que la Boîte Noire pour interpréter correctement les données reçues.
                        </p>
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