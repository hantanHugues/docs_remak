"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Separator } from "@/components/ui/separator"
import { Cpu, ArrowLeft, ArrowRight, AlertCircle, CheckCircle, Code, Zap, Cable } from "lucide-react"
import Link from "next/link"


export default function I2CPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Header */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-green-50 via-white to-emerald-50 dark:from-green-950/20 dark:via-background dark:to-emerald-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="/docs/semaine-1/electronique" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Circuit I2C</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-xl bg-gradient-to-r from-green-500 to-emerald-500 flex items-center justify-center">
                      <Cpu className="w-6 h-6 text-white" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Circuit I2C</h1>
                      <p className="text-muted-foreground">Communication série entre composants</p>
                    </div>
                  </div>

                  <div className="flex flex-wrap gap-2">
                    <Badge className="bg-green-100 text-green-700 border-green-200">Débutant</Badge>
                    <Badge variant="outline">1.5h de lecture</Badge>
                    <Badge variant="outline">Schémas inclus</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          {/* Navigation */}
          <PageNavigation />

          {/* Content */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              {/* Introduction */}
              <AnimatedSection animation="fade-up">
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Cpu className="w-5 h-5 text-green-600" />
                      Qu'est-ce que l'I2C ?
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="prose prose-gray dark:prose-invert max-w-none">
                    <p>
                      <strong>I2C</strong> (Inter-Integrated Circuit) est un protocole de communication série 
                      développé par Philips. Il permet de connecter plusieurs composants électroniques 
                      avec seulement <strong>2 fils</strong> : SDA (données) et SCL (horloge).
                    </p>
                    
                    <div className="grid md:grid-cols-3 gap-4 not-prose mt-6">
                      <div className="p-4 bg-green-50 dark:bg-green-950/20 rounded-lg text-center">
                        <Cable className="w-8 h-8 text-green-600 mx-auto mb-2" />
                        <h4 className="font-semibold text-green-900 dark:text-green-100 mb-1">2 fils seulement</h4>
                        <p className="text-xs text-green-700 dark:text-green-300">SDA + SCL</p>
                      </div>
                      <div className="p-4 bg-blue-50 dark:bg-blue-950/20 rounded-lg text-center">
                        <Zap className="w-8 h-8 text-blue-600 mx-auto mb-2" />
                        <h4 className="font-semibold text-blue-900 dark:text-blue-100 mb-1">Multi-maître</h4>
                        <p className="text-xs text-blue-700 dark:text-blue-300">Plusieurs contrôleurs</p>
                      </div>
                      <div className="p-4 bg-purple-50 dark:bg-purple-950/20 rounded-lg text-center">
                        <Cpu className="w-8 h-8 text-purple-600 mx-auto mb-2" />
                        <h4 className="font-semibold text-purple-900 dark:text-purple-100 mb-1">127 adresses</h4>
                        <p className="text-xs text-purple-700 dark:text-purple-300">Nombreux composants</p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Architecture */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Cable className="w-5 h-5 text-blue-600" />
                      Architecture et câblage
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-6">
                      <div>
                        <h3 className="text-lg font-semibold mb-3">Connexions physiques</h3>
                        <div className="bg-muted/50 p-4 rounded-lg">
                          <div className="grid md:grid-cols-2 gap-4">
                            <div>
                              <h4 className="font-medium mb-2 text-green-600">SDA (Serial Data)</h4>
                              <ul className="text-sm space-y-1">
                                <li>• Ligne bidirectionnelle pour les données</li>
                                <li>• Résistance de pull-up requise (4.7kΩ)</li>
                                <li>• Open-drain/Open-collector</li>
                              </ul>
                            </div>
                            <div>
                              <h4 className="font-medium mb-2 text-blue-600">SCL (Serial Clock)</h4>
                              <ul className="text-sm space-y-1">
                                <li>• Signal d'horloge généré par le maître</li>
                                <li>• Résistance de pull-up requise (4.7kΩ)</li>
                                <li>• Fréquence standard : 100kHz</li>
                              </ul>
                            </div>
                          </div>
                        </div>
                      </div>

                      <Separator />

                      <div>
                        <h3 className="text-lg font-semibold mb-3">Schéma de connexion</h3>
                        <div className="bg-gray-900 text-gray-100 p-6 rounded-lg font-mono text-sm">
                          <pre>{`                    +5V
                     |
                 4.7kΩ  4.7kΩ
                     |     |
    Arduino    ------+-----+------ Capteur 1
      SDA     -------+-----+------ Capteur 2  
      SCL     -------+-----+------ Capteur 3
      GND     -------+-----+------ ...
                     |     |
                    GND   GND`}</pre>
                        </div>
                        <p className="text-sm text-muted-foreground mt-2">
                          Tous les composants partagent les mêmes lignes SDA et SCL
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Protocole */}
              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Zap className="w-5 h-5 text-purple-600" />
                      Protocole de communication
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-6">
                      <div>
                        <h3 className="text-lg font-semibold mb-3">Séquence de communication</h3>
                        <div className="space-y-3">
                          <div className="flex items-center gap-3 p-3 bg-blue-50 dark:bg-blue-950/20 rounded-lg">
                            <div className="w-6 h-6 rounded-full bg-blue-600 text-white text-xs flex items-center justify-center font-bold">1</div>
                            <div>
                              <h4 className="font-medium">START</h4>
                              <p className="text-sm text-muted-foreground">SDA passe de HIGH à LOW pendant que SCL est HIGH</p>
                            </div>
                          </div>
                          
                          <div className="flex items-center gap-3 p-3 bg-green-50 dark:bg-green-950/20 rounded-lg">
                            <div className="w-6 h-6 rounded-full bg-green-600 text-white text-xs flex items-center justify-center font-bold">2</div>
                            <div>
                              <h4 className="font-medium">ADRESSE + R/W</h4>
                              <p className="text-sm text-muted-foreground">7 bits d'adresse + 1 bit Read/Write</p>
                            </div>
                          </div>
                          
                          <div className="flex items-center gap-3 p-3 bg-yellow-50 dark:bg-yellow-950/20 rounded-lg">
                            <div className="w-6 h-6 rounded-full bg-yellow-600 text-white text-xs flex items-center justify-center font-bold">3</div>
                            <div>
                              <h4 className="font-medium">ACK/NACK</h4>
                              <p className="text-sm text-muted-foreground">L'esclave confirme la réception</p>
                            </div>
                          </div>
                          
                          <div className="flex items-center gap-3 p-3 bg-purple-50 dark:bg-purple-950/20 rounded-lg">
                            <div className="w-6 h-6 rounded-full bg-purple-600 text-white text-xs flex items-center justify-center font-bold">4</div>
                            <div>
                              <h4 className="font-medium">DONNÉES</h4>
                              <p className="text-sm text-muted-foreground">Transmission des octets de données</p>
                            </div>
                          </div>
                          
                          <div className="flex items-center gap-3 p-3 bg-red-50 dark:bg-red-950/20 rounded-lg">
                            <div className="w-6 h-6 rounded-full bg-red-600 text-white text-xs flex items-center justify-center font-bold">5</div>
                            <div>
                              <h4 className="font-medium">STOP</h4>
                              <p className="text-sm text-muted-foreground">SDA passe de LOW à HIGH pendant que SCL est HIGH</p>
                            </div>
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Code pratique */}
              <AnimatedSection animation="fade-up" delay={300}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Code className="w-5 h-5 text-orange-600" />
                      Exemple pratique Arduino
                    </CardTitle>
                    <CardDescription>
                      Scanner I2C pour détecter les composants connectés
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-gray-900 rounded-lg p-4 overflow-x-auto">
                      <pre className="text-sm text-gray-100">
                        <code>{`#include <Wire.h>

void setup() {
  Wire.begin();        // Initialiser I2C en tant que maître
  Serial.begin(9600);
  Serial.println("Scanner I2C");
  Serial.println("Recherche de composants...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scan en cours...");

  // Tester toutes les adresses de 1 à 126
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Composant trouvé à l'adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Erreur inconnue à l'adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0) {
    Serial.println("Aucun composant I2C trouvé");
  } else {
    Serial.print(nDevices);
    Serial.println(" composant(s) trouvé(s)");
  }

  delay(5000); // Attendre 5 secondes avant le prochain scan
}`}</code>
                      </pre>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Conseils pratiques */}
              <AnimatedSection animation="fade-up" delay={400}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <AlertCircle className="w-5 h-5 text-orange-600" />
                      Conseils et bonnes pratiques
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Résistances de pull-up</h4>
                          <p className="text-sm text-muted-foreground">
                            Toujours utiliser des résistances de 4.7kΩ entre SDA/SCL et VCC
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Longueur des câbles</h4>
                          <p className="text-sm text-muted-foreground">
                            Limiter la longueur des câbles (&lt; 1m) pour éviter les interférences
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Gestion des erreurs</h4>
                          <p className="text-sm text-muted-foreground">
                            Toujours vérifier la valeur de retour de endTransmission()
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Adresses communes</h4>
                          <p className="text-sm text-muted-foreground">
                            MPU6050: 0x68, LCD I2C: 0x27, RTC DS3231: 0x68
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation footer */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-1/electronique/gyroscope">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Gyroscope & Accéléromètre
                  </Button>
                </Link>
                <Link href="/docs/semaine-1/electronique/lcd">
                  <Button>
                    Affichage LCD
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
