"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { PageNavigation } from "@/components/page-navigation";
import { AnimatedSection } from "@/components/animated-section"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Separator } from "@/components/ui/separator"
import { Monitor, ArrowLeft, AlertCircle, CheckCircle, Code, Zap, Tv } from "lucide-react"
import Link from "next/link"


export default function LCDPage() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Header */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-purple-50 via-white to-pink-50 dark:from-purple-950/20 dark:via-background dark:to-pink-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="/docs/semaine-1/electronique" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Affichage LCD</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-xl bg-gradient-to-r from-purple-500 to-pink-500 flex items-center justify-center">
                      <Monitor className="w-6 h-6 text-white" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Affichage LCD</h1>
                      <p className="text-muted-foreground">Interface utilisateur pour vos projets</p>
                    </div>
                  </div>

                  <div className="flex flex-wrap gap-2">
                    <Badge className="bg-purple-100 text-purple-700 border-purple-200">Débutant</Badge>
                    <Badge variant="outline">1h de lecture</Badge>
                    <Badge variant="outline">Code pratique</Badge>
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
                      <Monitor className="w-5 h-5 text-purple-600" />
                      Introduction aux écrans LCD
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="prose prose-gray dark:prose-invert max-w-none">
                    <p>
                      Les <strong>écrans LCD</strong> (Liquid Crystal Display) sont des composants essentiels 
                      pour créer une interface utilisateur dans vos projets robotiques. Ils permettent 
                      d'afficher des informations, des menus et des données en temps réel.
                    </p>
                    
                    <div className="grid md:grid-cols-2 gap-4 not-prose mt-6">
                      <div className="p-4 bg-purple-50 dark:bg-purple-950/20 rounded-lg">
                        <h4 className="font-semibold text-purple-900 dark:text-purple-100 mb-2">LCD 16x2</h4>
                        <p className="text-sm text-purple-700 dark:text-purple-300">
                          2 lignes de 16 caractères, idéal pour débuter
                        </p>
                      </div>
                      <div className="p-4 bg-blue-50 dark:bg-blue-950/20 rounded-lg">
                        <h4 className="font-semibold text-blue-900 dark:text-blue-100 mb-2">LCD I2C</h4>
                        <p className="text-sm text-blue-700 dark:text-blue-300">
                          Seulement 4 fils nécessaires (VCC, GND, SDA, SCL)
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Types d'écrans */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Tv className="w-5 h-5 text-blue-600" />
                      Types d'écrans LCD
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-6">
                      <div className="grid md:grid-cols-2 gap-6">
                        <div className="p-4 border rounded-lg">
                          <h3 className="text-lg font-semibold mb-3 text-green-600">LCD Parallèle (HD44780)</h3>
                          <div className="space-y-2 text-sm">
                            <p><strong>Connexions :</strong> 6-12 broches</p>
                            <p><strong>Avantages :</strong> Rapide, contrôle direct</p>
                            <p><strong>Inconvénients :</strong> Beaucoup de câblage</p>
                          </div>
                          <div className="mt-3 p-2 bg-green-50 dark:bg-green-950/20 rounded text-xs">
                            <strong>Broches :</strong> VSS, VDD, V0, RS, Enable, D4-D7
                          </div>
                        </div>

                        <div className="p-4 border rounded-lg">
                          <h3 className="text-lg font-semibold mb-3 text-blue-600">LCD I2C</h3>
                          <div className="space-y-2 text-sm">
                            <p><strong>Connexions :</strong> 4 broches seulement</p>
                            <p><strong>Avantages :</strong> Câblage simple, économise les pins</p>
                            <p><strong>Inconvénients :</strong> Légèrement plus lent</p>
                          </div>
                          <div className="mt-3 p-2 bg-blue-50 dark:bg-blue-950/20 rounded text-xs">
                            <strong>Broches :</strong> VCC, GND, SDA, SCL
                          </div>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Câblage */}
              <AnimatedSection animation="fade-up" delay={200}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Zap className="w-5 h-5 text-orange-600" />
                      Câblage et connexions
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-6">
                      <div>
                        <h3 className="text-lg font-semibold mb-3">LCD I2C (Recommandé)</h3>
                        <div className="bg-gray-900 text-gray-100 p-4 rounded-lg font-mono text-sm">
                          <pre>{`LCD I2C    →    Arduino
────────────────────────
VCC        →    5V
GND        →    GND
SDA        →    A4 (Uno) / 20 (Mega)
SCL        →    A5 (Uno) / 21 (Mega)`}</pre>
                        </div>
                      </div>

                      <Separator />

                      <div>
                        <h3 className="text-lg font-semibold mb-3">LCD Parallèle (6 fils)</h3>
                        <div className="bg-gray-900 text-gray-100 p-4 rounded-lg font-mono text-sm">
                          <pre>{`LCD Pin    →    Arduino
────────────────────────
VSS        →    GND
VDD        →    5V
V0         →    Potentiomètre (contraste)
RS         →    Pin 12
Enable     →    Pin 11
D4         →    Pin 5
D5         →    Pin 4
D6         →    Pin 3
D7         →    Pin 2`}</pre>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Code I2C */}
              <AnimatedSection animation="fade-up" delay={300}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Code className="w-5 h-5 text-green-600" />
                      Code pour LCD I2C
                    </CardTitle>
                    <CardDescription>
                      Exemple complet avec la bibliothèque LiquidCrystal_I2C
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="bg-gray-900 rounded-lg p-4 overflow-x-auto">
                      <pre className="text-sm text-gray-100">
                        <code>{`#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Créer un objet LCD (adresse, colonnes, lignes)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialiser l'écran LCD
  lcd.init();
  
  // Allumer le rétroéclairage
  lcd.backlight();
  
  // Afficher un message de bienvenue
  lcd.setCursor(0, 0);
  lcd.print("TRC Robotique");
  lcd.setCursor(0, 1);
  lcd.print("Pret a demarrer!");
  
  delay(2000);
}

void loop() {
  // Effacer l'écran
  lcd.clear();
  
  // Afficher l'heure de fonctionnement
  lcd.setCursor(0, 0);
  lcd.print("Temps: ");
  lcd.print(millis() / 1000);
  lcd.print("s");
  
  // Afficher un compteur
  static int compteur = 0;
  lcd.setCursor(0, 1);
  lcd.print("Compteur: ");
  lcd.print(compteur++);
  
  delay(1000);
  
  // Animation simple
  for(int i = 0; i < 16; i++) {
    lcd.setCursor(i, 1);
    lcd.print("*");
    delay(100);
  }
  
  delay(500);
}`}</code>
                      </pre>
                    </div>
                    
                    <div className="mt-4 p-4 bg-blue-50 dark:bg-blue-950/20 rounded-lg">
                      <h4 className="font-medium text-blue-900 dark:text-blue-100 mb-2">Installation de la bibliothèque</h4>
                      <p className="text-sm text-blue-700 dark:text-blue-300">
                        Dans l'IDE Arduino : Outils → Gérer les bibliothèques → Rechercher "LiquidCrystal I2C" → Installer
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Fonctions utiles */}
              <AnimatedSection animation="fade-up" delay={400}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <Code className="w-5 h-5 text-purple-600" />
                      Fonctions principales
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <div className="grid md:grid-cols-2 gap-4">
                        <div className="p-3 bg-muted/50 rounded-lg">
                          <h4 className="font-mono text-sm font-semibold mb-1">lcd.init()</h4>
                          <p className="text-xs text-muted-foreground">Initialise l'écran LCD</p>
                        </div>
                        
                        <div className="p-3 bg-muted/50 rounded-lg">
                          <h4 className="font-mono text-sm font-semibold mb-1">lcd.backlight()</h4>
                          <p className="text-xs text-muted-foreground">Active le rétroéclairage</p>
                        </div>
                        
                        <div className="p-3 bg-muted/50 rounded-lg">
                          <h4 className="font-mono text-sm font-semibold mb-1">lcd.clear()</h4>
                          <p className="text-xs text-muted-foreground">Efface tout l'écran</p>
                        </div>
                        
                        <div className="p-3 bg-muted/50 rounded-lg">
                          <h4 className="font-mono text-sm font-semibold mb-1">lcd.setCursor(x,y)</h4>
                          <p className="text-xs text-muted-foreground">Positionne le curseur</p>
                        </div>
                        
                        <div className="p-3 bg-muted/50 rounded-lg">
                          <h4 className="font-mono text-sm font-semibold mb-1">lcd.print()</h4>
                          <p className="text-xs text-muted-foreground">Affiche du texte</p>
                        </div>
                        
                        <div className="p-3 bg-muted/50 rounded-lg">
                          <h4 className="font-mono text-sm font-semibold mb-1">lcd.createChar()</h4>
                          <p className="text-xs text-muted-foreground">Crée des caractères personnalisés</p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Conseils pratiques */}
              <AnimatedSection animation="fade-up" delay={500}>
                <Card>
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                      <AlertCircle className="w-5 h-5 text-orange-600" />
                      Conseils et dépannage
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Trouver l'adresse I2C</h4>
                          <p className="text-sm text-muted-foreground">
                            Utilisez un scanner I2C si l'adresse 0x27 ne fonctionne pas (parfois 0x3F)
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Contraste</h4>
                          <p className="text-sm text-muted-foreground">
                            Si rien ne s'affiche, ajustez le potentiomètre de contraste sur le module I2C
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Alimentation</h4>
                          <p className="text-sm text-muted-foreground">
                            Vérifiez que l'écran reçoit bien 5V (certains modules acceptent 3.3V)
                          </p>
                        </div>
                      </div>
                      
                      <div className="flex items-start gap-3">
                        <CheckCircle className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
                        <div>
                          <h4 className="font-medium">Caractères spéciaux</h4>
                          <p className="text-sm text-muted-foreground">
                            Évitez les accents, utilisez createChar() pour des symboles personnalisés
                          </p>
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* Navigation footer */}
              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-1/electronique/i2c">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Circuit I2C
                  </Button>
                </Link>
                <Link href="/docs/semaine-1/electronique">
                  <Button>
                    Retour à l'Électronique
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
