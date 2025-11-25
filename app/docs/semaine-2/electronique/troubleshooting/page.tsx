"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import {
  ArrowLeft, ArrowRight, AlertTriangle, CloudUpload, GitCompareArrows, Activity, ShieldAlert
} from "lucide-react";
import Link from "next/link";
import { useMounted } from "@/hooks/use-mounted";


export default function TroubleshootingFAQFRPage() {
  const mounted = useMounted();
  
  if (!mounted) {
    return null;
  }

  return (
    <div className="min-h-screen bg-background">
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* En-tête */}
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-gradient-to-br from-yellow-50 via-white to-orange-50 dark:from-yellow-950/20 dark:via-background dark:to-orange-950/20">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="#" className="hover:text-foreground transition-colors">
                      Électronique
                    </Link>
                    <span>/</span>
                    <span>Dépannage (FAQ)</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <AlertTriangle className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Dépannage (FAQ) - Boîte Noire & Station de Contrôle</h1>
                      <p className="text-muted-foreground">Solutions aux problèmes courants rencontrés lors de l'assemblage et de l'utilisation.</p>
                    </div>
                  </div>

                   <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Dépannage</Badge>
                    <Badge variant="outline">FAQ</Badge>
                    <Badge variant="outline">Électronique</Badge>
                    <Badge variant="outline">Arduino</Badge>
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
              
              {/* SECTION 1 - Problèmes de Téléversement */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-red-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-red-50 to-transparent dark:from-red-950/20">
                    <CardTitle id="upload-issues" className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <CloudUpload className="w-6 h-6 text-red-600" />
                      1. Problèmes de Téléversement du Firmware
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : Erreur "avrdude: ser_open(): can't open device" ou "Port non trouvé."</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong> Vérifiez que le bon port COM/USB est sélectionné dans l'IDE Arduino (<code>Outils &gt; Port</code>). Assurez-vous que l'adaptateur USB-Série (ou l'Arduino Uno utilisé comme programmateur) est correctement connecté et que les pilotes (ex: FTDI, CH340) sont installés sur l'ordinateur.
                      </p>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : Erreur "stk500_getsync(): not in sync: resp=0x00".</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong> Ceci indique souvent un problème de communication entre l'IDE et le microcontrôleur.
                      </p>
                      <ul className="list-disc pl-5 mt-3 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                          <li>Vérifiez que la bonne carte est sélectionnée (<code>Outils &gt; Type de carte</code>).</li>
                          <li>Assurez-vous que l'ATmega328P est correctement câblé pour le téléversement (y compris les connexions TX/RX, Reset).</li>
                          <li>Essayez de maintenir le bouton "Reset" de l'Arduino (ou de débrancher/rebrancher GND sur l'ATmega autonome) juste avant que le téléversement ne commence (juste après la compilation).</li>
                      </ul>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : Le code compile mais rien ne se passe sur la carte.</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong> Vérifiez l'alimentation de la carte et les connexions <code>GND</code>. Si un ATmega328P autonome est utilisé, assurez-vous que le quartz de 16 MHz et ses condensateurs de 22 pF sont correctement connectés et que le bootloader est correctement gravé.
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 2 - Problèmes de Communication I2C */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-blue-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-blue-50 to-transparent dark:from-blue-950/20">
                    <CardTitle id="i2c-issues" className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <GitCompareArrows className="w-6 h-6 text-blue-600" />
                      2. Problèmes de Communication I2C
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : L'écran LCD de la Station de Contrôle reste vierge ou affiche des carrés.</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong>
                      </p>
                      <ul className="list-disc pl-5 mt-3 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                          <li>Vérifiez le câblage de l'écran LCD, en particulier SDA, SCL, VCC et GND.</li>
                          <li>Assurez-vous que le potentiomètre de contraste à l'arrière du module I2C de l'écran LCD est correctement ajusté (tournez avec un petit tournevis).</li>
                          <li>Vérifiez l'adresse I2C du module LCD. L'adresse par défaut dans le code (<code>0x27</code>) est la plus courante, mais certains modules utilisent <code>0x3F</code>. Un sketch "I2C scanner" (recherchez "Arduino I2C scanner" en ligne) peut être utilisé pour trouver l'adresse réelle de l'écran LCD.</li>
                      </ul>
                    </div>
                    
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : La Station de Contrôle n'affiche pas de données ou affiche des valeurs erronées.</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong>
                      </p>
                      <ul className="list-disc pl-5 mt-3 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                          <li>Assurez-vous que les broches SDA et SCL de la Boîte Noire et de la Station de Contrôle sont correctement connectées (SDA à SDA, SCL à SCL).</li>
                          <li className="font-bold text-red-600 dark:text-red-400">Les masses (GND) des deux modules doivent être interconnectées ! C'est une erreur I2C très courante.</li>
                          <li>Vérifiez que les firmwares des deux modules sont correctement téléversés et correspondent aux versions attendues.</li>
                          <li>Confirmez que <code>STATION_ADDRESS</code> est identique dans les deux codes.</li>
                          <li>Les résistances de pull-up (4.7kΩ) sur SDA et SCL sont-elles présentes ? Elles sont souvent intégrées dans les modules ou les cartes Arduino mais peuvent être nécessaires si l'ATmega est utilisé de manière autonome.</li>
                      </ul>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              {/* SECTION 3 - Problèmes du Capteur MPU-6050 */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-purple-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-purple-50 to-transparent dark:from-purple-950/20">
                    <CardTitle id="mpu6050-issues" className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <Activity className="w-6 h-6 text-purple-600" />
                      3. Problèmes du Capteur MPU-6050
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : Le MPU-6050 ne retourne pas de données ou affiche des valeurs fixes/erronées.</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong>
                      </p>
                      <ul className="list-disc pl-5 mt-3 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                          <li>Vérifiez le câblage du MPU-6050 (VCC, GND, SDA, SCL, INT).</li>
                          <li>Confirmez l'alimentation du MPU-6050 (VCC). Certains modules sont en 3.3V, d'autres ont un régulateur 5V intégré.</li>
                          <li>Assurez-vous que l'adresse I2C du MPU-6050 est correcte (<code>0x68</code> ou <code>0x69</code> si la broche AD0 est tirée à HIGH). Le code utilise <code>0x68</code>.</li>
                      </ul>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              {/* SECTION 4 - Problèmes de Détection de Crash et EEPROM */}
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border-l-4 border-l-orange-500 mt-12">
                  <CardHeader className="bg-gradient-to-r from-orange-50 to-transparent dark:from-orange-950/20">
                    <CardTitle id="crash-eeprom-issues" className="flex items-center gap-3 scroll-mt-24 text-xl">
                      <ShieldAlert className="w-6 h-6 text-orange-600" />
                      4. Problèmes de Détection de Crash et d'EEPROM
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-6 pt-6">

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : Le système ne détecte pas les crashs ou détecte des "faux positifs".</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong> Le seuil de détection <code>CRASH_THRESHOLD_G</code> peut nécessiter un ajustement. Cette valeur dépend de la sensibilité souhaitée et du type d'impact à simuler. Commencez avec des valeurs plus basses et augmentez progressivement.
                      </p>
                      <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">
                        Vérifiez la stabilité du capteur. Des vibrations excessives pendant un vol normal peuvent être interprétées comme des faux positifs.
                      </p>
                    </div>

                    <div className="bg-white dark:bg-gray-900/50 border border-gray-200 dark:border-gray-700 rounded-lg p-6">
                      <h4 className="font-semibold text-gray-900 dark:text-gray-100 mb-2">Q : Les données ne sont pas sauvegardées dans l'EEPROM ou les données récupérées sont corrompues.</h4>
                      <p className="text-sm text-gray-600 dark:text-gray-400">
                        <strong>R :</strong>
                      </p>
                      <ul className="list-disc pl-5 mt-3 space-y-2 text-sm text-gray-600 dark:text-gray-400">
                          <li>Assurez-vous que la bibliothèque standard Arduino EEPROM est utilisée.</li>
                          <li>Vérifiez la taille de votre structure <code>FlightData</code> et la gestion des adresses de l'EEPROM dans le code. Les fonctions <code>EEPROM.put()</code> et <code>EEPROM.get()</code> gèrent automatiquement la taille de la structure, mais les adresses (<code>EEPROM_CRASH_FLAG_ADDR</code>, <code>EEPROM_CRASH_ADDR_ADDR</code>) doivent être correctement définies pour éviter les chevauchements.</li>
                      </ul>
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