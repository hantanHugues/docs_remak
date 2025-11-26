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
  ArrowLeft, ArrowRight, BookOpen, FileText, Cpu, GitCompareArrows, PlayCircle, Settings, ExternalLink, Code
} from "lucide-react";
import Link from "next/link";

export default function IntelligentConveyorDocPage() {
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
                    <span>Projet Convoyeur Intelligent</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Settings className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation du Projet Convoyeur Intelligent</h1>
                      <p className="text-muted-foreground">Contrôle d'un système de convoyeur avec détection de couleur et communication I2C.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">PlatformIO</Badge>
                    <Badge variant="outline">Arduino Nano</Badge>
                    <Badge variant="outline">ESP32</Badge>
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
            <div className="max-w-4xl mx-auto space-y-8">
              
              <AnimatedSection animation="fade-up">
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      1. Vue d'ensemble
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <div className="text-gray-700 dark:text-gray-300 leading-relaxed">
                      <p>
                        Ce projet contrôle un système de convoyeur capable de détecter la couleur d'un objet et de communiquer cette information à un backend distant. Le code a été scindé en deux applications distinctes pour répartir les tâches entre un Arduino Nano et un ESP32, communiquant via le protocole I2C.
                      </p>
                      <p className="mt-4">
                        Ce document décrit les trois principaux fichiers de code du projet :
                      </p>
                      <ul className="list-disc pl-5 my-4 space-y-1 text-sm">
                        <li><code>src/main.cpp</code> : Le code original pour les testes rapides (sans I2C car mon ordinateur n'a qu'un port usb)</li>
                        <li><code>src/arduino_nano_master.cpp</code> : Le nouveau code pour l'Arduino Nano (Maître I2C).</li>
                        <li><code>src/esp32_slave.cpp</code> : Le nouveau code pour l'ESP32 (Esclave I2C).</li>
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
                      <FileText className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      2. Description des Fichiers de Code
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 p-6">
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-2">a. <code>src/main.cpp</code> (Code Original)</h3>
                      <ul className="list-disc pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>Rôle</strong> : Ce fichier contient la logique complète et originale du projet. Il gère à la fois la détection de couleur, la gestion des capteurs LDR, le contrôle du moteur, la connexion WiFi et l'envoi des données au serveur web.</li>
                        <li><strong>Microcontrôleur Cible</strong> : ESP32 Dev Kit V1.</li>
                        <li><strong>Principe</strong> : Bien que fonctionnel, ce code centralise toutes les tâches sur un seul microcontrôleur. Il sert maintenant de référence ou de version de base pour un fonctionnement sur un unique ESP32.</li>
                        <li><strong>Dépendances</strong> : <code>Adafruit_TCS34725</code>, <code>WiFi</code>, <code>HTTPClient</code>.</li>
                      </ul>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-2">b. <code>src/arduino_nano_master.cpp</code> (Nouveau - Maître I2C)</h3>
                      <ul className="list-disc pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>Rôle</strong> : Ce code est dédié à la détection de couleur. Il est responsable de l'initialisation du capteur TCS34725, de la calibration, de l'identification et de la transmission d'un identifiant simple à l'ESP32 via I2C.</li>
                        <li><strong>Microcontrôleur Cible</strong> : Arduino Nano.</li>
                        <li><strong>Logique de fonctionnement</strong> :
                          <ol className="list-decimal pl-5 mt-1">
                            <li>Initialise le capteur et le bus I2C en mode "Maître".</li>
                            <li>Propose une phase de calibration via le moniteur série.</li>
                            <li>Lit les données du capteur en boucle.</li>
                            <li>Identifie la couleur (Rouge, Vert, Bleu, Jaune).</li>
                            <li>Envoie un caractère correspondant à l'ESP32 (`'R'`, `'V'`, `'B'`, `'J'`).</li>
                          </ol>
                        </li>
                        <li><strong>Dépendances</strong> : <code>Adafruit_TCS34725</code>, <code>Wire</code>.</li>
                      </ul>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                      <h3 className="font-medium text-gray-900 dark:text-gray-100 mb-2">c. <code>src/esp32_slave.cpp</code> (Nouveau - Esclave I2C)</h3>
                      <ul className="list-disc pl-5 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>Rôle</strong> : Ce code gère la communication. Il reçoit les données de l'Arduino Nano, se connecte au réseau WiFi et envoie les informations de couleur au backend.</li>
                        <li><strong>Microcontrôleur Cible</strong> : ESP32 Dev Kit V1.</li>
                        <li><strong>Logique de fonctionnement</strong> :
                          <ol className="list-decimal pl-5 mt-1">
                            <li>Initialise la connexion WiFi et le bus I2C en mode "Esclave".</li>
                            <li>Attend passivement les données de l'Arduino.</li>
                            <li>Une fonction d'interruption stocke le caractère reçu.</li>
                            <li>La boucle principale vérifie si une nouvelle donnée est arrivée.</li>
                            <li>Si oui, elle traduit le caractère en nom de couleur complet.</li>
                            <li>Elle envoie ensuite ce nom au serveur distant via une requête HTTP POST.</li>
                          </ol>
                        </li>
                        <li><strong>Dépendances</strong> : <code>WiFi</code>, <code>HTTPClient</code>, <code>Wire</code>.</li>
                      </ul>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      3. Guide d'Utilisation du Système Séparé
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div>
                      <h3 className="font-medium text-lg text-gray-800 dark:text-gray-200 mb-2">a. Connexions Matérielles (I2C)</h3>
                      <p className="text-gray-700 dark:text-gray-300 mb-4">Pour que les deux cartes communiquent, vous devez les relier via le bus I2C :</p>
                      <ul className="list-disc pl-5 space-y-2 text-gray-600 dark:text-gray-400">
                        <li><strong>SDA (Serial Data)</strong> : Reliez la broche <code>A4</code> de l'Arduino Nano à la broche <code>GPIO 21</code> (SDA) de l'ESP32.</li>
                        <li><strong>SCL (Serial Clock)</strong> : Reliez la broche <code>A5</code> de l'Arduino Nano à la broche <code>GPIO 22</code> (SCL) de l'ESP32.</li>
                        <li><strong>GND (Masse)</strong> : Reliez une broche <code>GND</code> de l'Arduino Nano à une broche <code>GND</code> de l'ESP32 pour assurer une référence de tension commune.</li>
                      </ul>
                      <p className="mt-4 text-sm bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3">
                        <em>N'oubliez pas de brancher le capteur de couleur TCS34725 à l'Arduino Nano.</em>
                      </p>
                    </div>
                    <div>
                      <h3 className="font-medium text-lg text-gray-800 dark:text-gray-200 mb-2">b. Configuration du Projet (platformio.ini)</h3>
                      <p className="text-gray-700 dark:text-gray-300 mb-4">Le fichier <code>platformio.ini</code> est maintenant configuré avec deux environnements distincts :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4 text-gray-600 dark:text-gray-400">
                        <li><code>[env:esp32_slave]</code> : Pour l'ESP32.</li>
                        <li><code>[env:arduino_nano_master]</code> : Pour l'Arduino Nano.</li>
                      </ul>
                      <h4 className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Actions requises :</h4>
                      <ol className="list-decimal pl-5 text-sm text-gray-600 dark:text-gray-400">
                        <li><strong>WiFi</strong> : Dans <code>src/esp32_slave.cpp</code>, remplacez <code>"VOTRE_SSID_WIFI"</code> et <code>"VOTRE_MOT_DE_PASSE_WIFI"</code>.</li>
                        <li><strong>Ports Série</strong> : Dans <code>platformio.ini</code>, vérifiez que les <code>upload_port</code> et <code>monitor_port</code> correspondent à vos cartes.</li>
                      </ol>
                    </div>
                    <div>
                      <h3 className="font-medium text-lg text-gray-800 dark:text-gray-200 mb-2">c. Compilation et Téléversement</h3>
                      <p className="text-gray-700 dark:text-gray-300 mb-4">Utilisez l'interface de PlatformIO dans VSCode ou le terminal :</p>
                      <p className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Pour l'ESP32 :</p>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`pio run -e esp32_slave --target upload`}</code></pre></div>
                      <p className="font-semibold text-gray-800 dark:text-gray-200 mb-2">Pour l'Arduino Nano :</p>
                      <div className="bg-gray-900 dark:bg-gray-950 rounded-lg p-4 mb-4 overflow-x-auto relative border border-gray-700"><pre className="text-sm text-gray-100"><code>{`pio run -e arduino_nano_master --target upload`}</code></pre></div>
                      <p className="text-gray-700 dark:text-gray-300">Une fois les deux codes téléversés, vous pouvez ouvrir deux moniteurs série pour observer les logs de chaque carte.</p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Médias et Liens Externes
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div className="aspect-video rounded-lg overflow-hidden border border-gray-200 dark:border-gray-700">
                      <iframe width="100%" height="100%" src="https://www.youtube.com/embed/nkq-DGI4sTo?si=BcFLYtN7oNogi4go" title="YouTube video player" frameBorder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowFullScreen></iframe>
                    </div>
                    <div className="aspect-video rounded-lg overflow-hidden border border-gray-200 dark:border-gray-700">
                      <iframe width="100%" height="100%" src="https://www.youtube.com/embed/zXMRKMIum10?si=mJmQTG8b7E_M-XHL" title="YouTube video player" frameBorder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowFullScreen></iframe>
                    </div>
                    <Button asChild>
                      <Link href="https://github.com/hantanHugues/esp_convoyeur_platrformIO" target="_blank" rel="noopener noreferrer">
                        <ExternalLink className="w-4 h-4 mr-2" />
                        Voir le projet sur GitHub
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
