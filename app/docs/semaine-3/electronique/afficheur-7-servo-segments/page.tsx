"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, Code, Settings, List, Target, HardHat, GitBranch, PlayCircle
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';
import { useMounted } from "@/hooks/use-mounted";

export default function ServoDisplayDocPage() {
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
          <AnimatedSection animation="fade-in">
            <section className="relative py-12 bg-white dark:bg-gray-900 border-b border-gray-200 dark:border-gray-700">
              <div className="container mx-auto px-4">
                <div className="max-w-4xl mx-auto">
                  <div className="flex items-center gap-2 text-sm text-muted-foreground mb-4">
                    <Link href="/docs/semaine-3/electronique" className="hover:text-foreground transition-colors">
                      Documentation Électronique
                    </Link>
                    <span>/</span>
                    <span>Projet Afficheur 7 Segments</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <Settings className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation Complète du Projet "Afficheur 7 Segments à Servomoteurs"</h1>
                      <p className="text-muted-foreground">Date : 28 Juin 2025 | Équipe : IFRI - Team Électronique</p>
                    </div>
                  </div>

                  <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Projet Complet</Badge>
                    <Badge variant="outline">Arduino</Badge>
                    <Badge variant="outline">Électronique</Badge>
                    <Badge variant="outline">Servomoteurs</Badge>
                  </div>
                </div>
              </div>
            </section>
          </AnimatedSection>

          <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
            <div className="container mx-auto px-4">
              <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
                <Link href="/docs/semaine-3/electronique">
                  <Button variant="ghost" size="sm">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Électronique Semaine 3
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/electronique/code-7-servo-segments">
                  <Button variant="ghost" size="sm">
                    Code 7 servo Segments
                    <ArrowRight className="w-4 h-4 ml-2" />
                  </Button>
                </Link>
              </div>
            </div>
          </div>

          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              
              <AnimatedSection animation="fade-up">
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <List className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Table des Matières
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <ol className="list-decimal pl-5 space-y-2 text-blue-600 dark:text-blue-400">
                      <li><a href="#1-introduction--vision-du-projet-et-objectifs-initiaux" className="hover:underline">Introduction : Vision du Projet et Objectifs Initiaux</a></li>
                      <li><a href="#2-conception-matérielle--les-choix-techniques-et-la-réalisation-physique" className="hover:underline">Conception Matérielle : Les Choix Techniques et la Réalisation Physique</a></li>
                      <li><a href="#3-conception-logicielle--les-stratégies-de-programmation" className="hover:underline">Conception Logicielle : Les Stratégies de Programmation</a></li>
                      <li><a href="#4-mise-en-œuvre--programmation-et-calibration-précise" className="hover:underline">Mise en Œuvre : Programmation et Calibration Précise</a></li>
                      <li><a href="#5-réalisation-finale-et-démonstration" className="hover:underline">Réalisation Finale et Démonstration</a></li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <Separator className="border-gray-200 dark:border-gray-700" />

              <div className="space-y-12">

                <AnimatedSection animation="fade-up" delay={50}>
                  <Card id="1-introduction--vision-du-projet-et-objectifs-initiaux" className="border border-gray-200 dark:border-gray-700 scroll-mt-24" suppressHydrationWarning>
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        1. Introduction : Vision du Projet et Objectifs Initiaux
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 text-gray-700 dark:text-gray-300 leading-relaxed space-y-4">
                      <p>Ce document retrace le parcours de notre projet d'électronique pour le TEKBOT ROBOTICS CHALLENGE 2025 : la réalisation d'un <strong>afficheur 7 segments d'un genre nouveau</strong>. L'idée principale était de s'affranchir des afficheurs lumineux traditionnels pour innover et démontrer notre aptitude à relever des défis techniques originaux.</p>
                      <p>Nous avons choisi de matérialiser chaque segment de l'afficheur par un <strong>servomoteur</strong>. L'objectif est de contrôler ces 7 servomoteurs de manière synchronisée pour afficher les chiffres de 0 à 9, avec un décompte de 9 à 0.</p>
                      <p className="bg-gray-50 dark:bg-gray-900/30 border border-gray-200 dark:border-gray-700 rounded p-3 text-sm">Le défi technique majeur imposé par le challenge et relevé avec succès est la conception d'un système <strong>sans utilisation de la fonction bloquante <code>delay()</code></strong> dans le code Arduino. Cette approche non bloquante garantit une réactivité optimale et permet l'intégration future d'autres fonctionnalités sans compromettre la fluidité du comptage.</p>
                      <p>Le concept clé de notre réalisation physique est une <strong>structure imprimée en 3D formant la façade de l'afficheur</strong>, avec des ouvertures hexagonales pour chaque segment. Chaque segment est une pièce mobile distincte, positionnée derrière son ouverture et connectée à un servomoteur par un fil de fer.</p>
                      <ul className="list-disc pl-5 space-y-2">
                        <li>Lorsque le segment est <strong>inactif</strong> (non concerné par le chiffre), le servomoteur tire le fil de fer, faisant rentrer le segment à l'intérieur du support. Sa face avant est alors <strong>confondue</strong> avec la surface du grand rectangle support, le rendant discret et visuellement intégré.</li>
                        <li>Lorsque le segment est <strong>actif</strong> (doit faire partie du chiffre), le servomoteur pousse le fil de fer, faisant "sortir" le segment vers l'avant. Sa face avant est alors <strong>en relief</strong> par rapport au support, le rendant clairement visible et créant l'affichage du chiffre.</li>
                      </ul>
                      <p>Cette approche innovante offre un rendu visuel dynamique et tactile, où les chiffres "émergent" littéralement de l'afficheur.</p>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card id="2-conception-matérielle--les-choix-techniques-et-la-réalisation-physique" className="border border-gray-200 dark:border-gray-700 scroll-mt-24" suppressHydrationWarning>
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <HardHat className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        2. Conception Matérielle : Les Choix Techniques et la Réalisation Physique
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-6">
                      <h3 id="21-le-microcontrôleur--atmega328p-nu" className="font-semibold text-lg text-gray-800 dark:text-gray-200">2.1. Le Microcontrôleur : ATmega328P nu</h3>
                      <p className="text-gray-700 dark:text-gray-300">Le cœur de notre système est le microcontrôleur <strong>ATmega328P</strong> en boîtier DIP-28. Ce choix s'est imposé pour plusieurs raisons :</p>
                      <ul className="list-disc pl-5 text-sm text-gray-600 dark:text-gray-400"><li><strong>Conformité au Challenge</strong>, <strong>Flexibilité</strong>, <strong>Coût</strong>, et <strong>Familiarité</strong>.</li></ul>
                      
                      <h3 id="22-le-contrôleur-de-servomoteurs--pca9685" className="font-semibold text-lg text-gray-800 dark:text-gray-200">2.2. Le Contrôleur de Servomoteurs : PCA9685</h3>
                      <p className="text-gray-700 dark:text-gray-300">Pour piloter les 7 servomoteurs, nous avons opté pour le module <strong>PCA9685</strong>. Il décharge le microcontrôleur de la génération des signaux PWM et communique via I2C.</p>
                      <Image src="/Documentation/semaine-3/electronique/images_vids/pca9685.jpg" alt="Module PCA9685" width={300} height={200} className="rounded-md border mx-auto" />
                      <Image src="/Documentation/semaine-3/electronique/images_vids/pca9685-puce-servos.png" alt="Schéma KiCad" width={700} height={400} className="rounded-md border mx-auto" />

                      <h3 id="23-lalimentation--module-mp1584en" className="font-semibold text-lg text-gray-800 dark:text-gray-200">2.3. L'Alimentation : Module MP1584EN</h3>
                      <p className="text-gray-700 dark:text-gray-300">Alimentation par batterie LiPo, régulée par un module DC-DC step-down <strong>MP1584EN</strong> pour son efficacité énergétique. Nous avons créé un footprint KiCad personnalisé pour ce module.</p>
                      <div className="grid md:grid-cols-2 gap-4"><Image src="/Documentation/semaine-3/electronique/images_vids/dimensions_dcdc.png" alt="Dimensions MP1584EN" width={300} height={200} className="rounded-md border" /><Image src="/Documentation/semaine-3/electronique/images_vids/mp1584en_footprint_kicad.png" alt="Footprint KiCad MP1584EN" width={300} height={200} className="rounded-md border" /></div>
                      
                      <h3 id="24-la-conception-physique-de-lafficheur" className="font-semibold text-lg text-gray-800 dark:text-gray-200">2.4. La Conception Physique de l'Afficheur</h3>
                      <p className="text-gray-700 dark:text-gray-300">Structure principale imprimée en 3D. Chaque segment mobile est connecté à un servo par un fil de fer pour un effet de relief.</p>
                      <div className="grid md:grid-cols-2 gap-4"><Image src="/Documentation/semaine-3/electronique/images_vids/face_afficheur1.jpeg" alt="Vue de face" width={300} height={200} className="rounded-md border" /><Image src="/Documentation/semaine-3/electronique/images_vids/lateral_afficheur.jpeg" alt="Vue latérale" width={300} height={200} className="rounded-md border" /></div>
                      
                      <h3 id="25-connectique-et-simplifications" className="font-semibold text-lg text-gray-800 dark:text-gray-200">2.5. Connectique et Simplifications</h3>
                      <p className="text-gray-700 dark:text-gray-300">Utilisation de connecteurs pour la modularité. Header ISP et LED RGB retirés pour se concentrer sur l'essentiel.</p>

                      <h3 id="26-réalisation-du-circuit-sur-veroboard" className="font-semibold text-lg text-gray-800 dark:text-gray-200">2.6. Réalisation du Circuit sur Veroboard</h3>
                      <p className="text-gray-700 dark:text-gray-300">Montage sur <strong>veroboard</strong> en raison des contraintes de temps.</p>
                      <div className="grid md:grid-cols-2 gap-4"><Image src="/Documentation/semaine-3/electronique/images_vids/circuit_minimal.png" alt="Circuit minimal KiCad" width={300} height={200} className="rounded-md border" /><Image src="/Documentation/semaine-3/electronique/images_vids/pcb.png" alt="PCB KiCad" width={300} height={200} className="rounded-md border" /></div>
                    </CardContent>
                  </Card>
                </AnimatedSection>
                
                <AnimatedSection animation="fade-up" delay={100}>
                  <Card id="3-conception-logicielle--les-stratégies-de-programmation" className="border border-gray-200 dark:border-gray-700 scroll-mt-24" suppressHydrationWarning>
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <GitBranch className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        3. Conception Logicielle : Les Stratégies de Programmation
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-6">
                      <h3 id="31-gestion-du-timing-non-bloquant-millis" className="font-semibold text-lg text-gray-800 dark:text-gray-200">3.1. Gestion du Timing Non Bloquant (<code>millis()</code>)</h3>
                      <p className="text-gray-700 dark:text-gray-300">Solution `millis()` utilisée pour des temporisations non bloquantes, garantissant la réactivité du microcontrôleur.</p>
                      <h3 id="32-structuration-du-code" className="font-semibold text-lg text-gray-800 dark:text-gray-200">3.2. Structuration du Code</h3>
                      <p className="text-gray-700 dark:text-gray-300">Code modulaire avec fonctions dédiées (`setSegmentState`, `displayDigit`) et un tableau de motifs (`digitPatterns`).</p>
                      <h3 id="33-contrôle-des-servos-et-valeurs-calibrées" className="font-semibold text-lg text-gray-800 dark:text-gray-200">3.3. Contrôle des Servos et Valeurs Calibrées</h3>
                      <p className="text-gray-700 dark:text-gray-300">Bibliothèque `Adafruit_PWMServoDriver` utilisée. Les positions sont définies par des "ticks" PWM, avec des valeurs calibrées `SERVO_PULSE_EXTENDED` et `SERVO_PULSE_RETRACTED` pour chaque segment.</p>
                      <h3 id="34-logique-de-comptage" className="font-semibold text-lg text-gray-800 dark:text-gray-200">3.4. Logique de Comptage</h3>
                      <p className="text-gray-700 dark:text-gray-300">Algorithme de comptage/décompte de 0 à 9 et de 9 à 0. La LED RGB a été retirée du design final.</p>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={100}>
                  <Card id="4-mise-en-œuvre--programmation-et-calibration-précise" className="border border-gray-200 dark:border-gray-700 scroll-mt-24" suppressHydrationWarning>
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <Code className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        4. Mise en Œuvre : Programmation et Calibration Précise
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-6">
                      <h3 id="41-le-défi--programmer-un-atmega328p-nu" className="font-semibold text-lg text-gray-800 dark:text-gray-200">4.1. Le Défi : Programmer un ATmega328P nu</h3>
                      <p className="text-gray-700 dark:text-gray-300">Un ATmega328P neuf nécessite une méthode de programmation spécifique car il n'a ni bootloader Arduino, ni interface USB-série.</p>
                      <h3 id="42-option-retenue--utilisation-dune-carte-arduino-uno-comme-programmeur-temporaire" className="font-semibold text-lg text-gray-800 dark:text-gray-200">4.2. Option Retenue : Utilisation d'une carte Arduino Uno comme Programmeur Temporaire</h3>
                      <p className="text-gray-700 dark:text-gray-300">Processus simple consistant à insérer notre puce dans le support d'une Uno, téléverser le code, puis remettre la puce sur le veroboard.</p>
                      <h3 id="43-tests-unitaires-et-calibration-individuelle-des-segments" className="font-semibold text-lg text-gray-800 dark:text-gray-200">4.3. Tests Unitaires et Calibration Individuelle des Segments</h3>
                      <p className="text-gray-700 dark:text-gray-300">Étape fondamentale utilisant un code de calibration spécifique pour ajuster et enregistrer itérativement les valeurs de pulse "étendu" et "rétracté" pour chaque servo, garantissant un affichage net.</p>
                      <h3 id="44-implications-pour-la-programmation-sur-veroboard" className="font-semibold text-lg text-gray-800 dark:text-gray-200">4.4. Implications pour la Programmation sur Veroboard</h3>
                      <p className="text-gray-700 dark:text-gray-300">L'absence de header ISP sur le veroboard final signifie que les mises à jour logicielles nécessitent de retirer l'ATmega328P pour le reprogrammer.</p>
                    </CardContent>
                  </Card>
                </AnimatedSection>

                <AnimatedSection animation="fade-up" delay={100}>
                  <Card id="5-réalisation-finale-et-démonstration" className="border border-gray-200 dark:border-gray-700 scroll-mt-24" suppressHydrationWarning>
                    <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                      <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                        <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                        5. Réalisation Finale et Démonstration
                      </CardTitle>
                    </CardHeader>
                    <CardContent className="p-6 space-y-6">
                      <h3 id="51-intégration-matérielle-et-assemblage-final" className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.1. Intégration Matérielle et Assemblage Final</h3>
                      <p className="text-gray-700 dark:text-gray-300">Le circuit sur veroboard a été intégré à la structure 3D, et les composants connectés.</p>
                      <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
                        <Image src="/Documentation/semaine-3/electronique/images_vids/image_totale_1.jpeg" alt="Branchement final 1" width={200} height={150} className="rounded-md border" />
                        <Image src="/Documentation/semaine-3/electronique/images_vids/image.png" alt="Branchement final 2" width={200} height={150} className="rounded-md border" />
                        <Image src="/Documentation/semaine-3/electronique/images_vids/image_copy.png" alt="Branchement final 3" width={200} height={150} className="rounded-md border" />
                        <Image src="/Documentation/semaine-3/electronique/images_vids/image_copy1.png" alt="Branchement final 4" width={200} height={150} className="rounded-md border" />
                        <Image src="/Documentation/semaine-3/electronique/images_vids/image_copy2.png" alt="Branchement final 5" width={200} height={150} className="rounded-md border" />
                      </div>
                      
                      <h3 id="52-fonctionnement-du-système-complet" className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.2. Fonctionnement du Système Complet</h3>
                      <p className="text-gray-700 dark:text-gray-300">Le système final effectue un cycle de comptage fluide de 0 à 9 et inversement, avec des transitions nettes grâce aux valeurs calibrées et au code non bloquant.</p>
                      
                      <h3 id="53-conclusion-et-bilan" className="font-semibold text-lg text-gray-800 dark:text-gray-200">5.3. Conclusion et Bilan</h3>
                      <p className="text-gray-700 dark:text-gray-300 leading-relaxed">
                        Le projet d'afficheur 7 segments à servomoteurs a été une réussite, même si certains obstacles techniques nous ont empêchés d'achever totalement le prototype. L'absence de résistance pour le reset sur le veroboard, ainsi que la faible résistance mécanique des fils de fer reliant les moteurs aux segments, ont constitué des freins majeurs à la finalisation complète du système. Malgré ces défis — utilisation d'un ATmega328P nu, contraintes de temps, et nécessité d'une calibration mécanique précise — nous avons su concevoir et réaliser un prototype fonctionnel et innovant. L'approche non bloquante du code et la rigueur apportée à la calibration ont été des facteurs clés de réussite. Ce projet illustre notre capacité à transformer une idée originale en réalisation concrète, en relevant des défis matériels et logiciels complexes.
                      </p>
                    </CardContent>
                  </Card>
                </AnimatedSection>
              </div>

              <div className="flex items-center justify-between pt-8 border-t">
                <Link href="/docs/semaine-3/electronique">
                  <Button variant="outline">
                    <ArrowLeft className="w-4 h-4 mr-2" />
                    Électronique Semaine 3
                  </Button>
                </Link>
                <Link href="/docs/semaine-3/electronique/code-7-servo-segments">
                  <Button>
                    Code 7 servo Segments
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