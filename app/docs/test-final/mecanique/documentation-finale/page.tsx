"use client";

import { Navbar } from "@/components/navbar";
import { DocsSidebarWrapper } from "@/components/docs-sidebar-wrapper";
import { AnimatedSection } from "@/components/animated-section";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import {
  ArrowLeft, ArrowRight, HardHat, Cog, BookOpen, Download, PlayCircle, FolderArchive, Target, CheckCircle, BarChart2, AlertTriangle, ExternalLink, FileText, Settings, Layers, GitBranch, Cpu
} from "lucide-react";
import Link from "next/link";
import Image from 'next/image';

// Custom Header Component to match the HTML structure
const ProjectHeader = () => (
  <header className="flex items-center gap-4 p-4 bg-white dark:bg-gray-900 border-b border-gray-200 dark:border-gray-800">
    <div className="w-12 h-12 bg-primary text-primary-foreground flex items-center justify-center rounded-md text-2xl font-bold">WF</div>
    <div>
      <h1 className="text-xl font-bold text-gray-900 dark:text-gray-100">WasteFlow — Documentation mécanique</h1>
      <p className="text-sm text-muted-foreground">Système de convoyeur intelligent — Tekbot Robotics Challenge 2025</p>
    </div>
  </header>
);

// Custom Hero Component
const HeroSection = () => (
  <div className="relative bg-gray-800 text-white py-20 px-4 text-center my-8 rounded-lg overflow-hidden">
    <div className="absolute inset-0 bg-black opacity-50 z-0"></div>
    <div className="relative z-10">
      <h1 className="text-5xl font-extrabold mb-4">WasteFlow</h1>
      <p className="text-lg mb-8">
        <strong><em>Triez intelligemment, assemblez facilement.</em></strong><br /><br />
        Un convoyeur modulaire 3D qui détecte, trie et oriente automatiquement vos pièces pour le Tekbot Robotics Challenge 2025.
      </p>
      <Button asChild size="lg">
        <a href="#contexte-objectifs">Découvrir le projet</a>
      </Button>
    </div>
  </div>
);

export default function WasteFlowMechanicsDocPage() {
  return (
    <div className="min-h-screen bg-background" suppressHydrationWarning={true}>
      <Navbar />

      <div className="flex pt-16 md:pt-20">
        <DocsSidebarWrapper />

        <main className="flex-1 min-w-0">
          {/* Le Header et Hero sont placés directement dans le main pour correspondre au HTML */}
          <div className="container mx-auto px-4 py-8">
            <div className="max-w-4xl mx-auto space-y-8">
              <ProjectHeader />
              <HeroSection />

              {/* SECTION 1 */}
              <AnimatedSection animation="fade-up">
                <Card id="contexte-objectifs" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 1 — Contexte & objectifs
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 text-gray-700 dark:text-gray-300 leading-relaxed space-y-4">
                    <p>Dans le cadre du Tekbot Robotics Challenge 2025, une entreprise innovante souhaite déployer un convoyeur intelligent pour automatiser le tri de déchets sous forme de cubes colorés (vert, jaune, rouge, bleu). Ce projet multidisciplinaire mobilise trois équipes : électronique, informatique et mécanique, qui doivent collaborer étroitement pour concevoir un système complet et fonctionnel.</p>
                    <p>Le système doit détecter la présence des déchets, les identifier par couleur et les orienter vers la benne adéquate, avec un suivi en temps réel via une interface web intuitive.</p>
                    <p>La mécanique est responsable de la conception de la structure physique du convoyeur. Elle doit garantir la robustesse, la stabilité, la précision des déplacements, ainsi que la modularité pour faciliter l’assemblage, la maintenance et les évolutions futures du système. Le design mécanique doit aussi prendre en compte les contraintes liées à l’intégration des composants électroniques et à l’impression 3D.</p>
                    <p><strong>Les objectifs mécaniques principaux sont :</strong></p>
                    <ul className="list-disc pl-5 space-y-2">
                      <li>respecter les dimensions du convoyeur (650 mm de long, 100 mm de haut) pour une intégration optimale</li>
                      <li>garantir la compatibilité avec les composants électroniques et l’alimentation par batterie lithium</li>
                      <li>concevoir une structure modulaire, robuste et imprimable en 3D</li>
                      <li>documenter rigoureusement chaque étape de la conception mécanique pour assurer traçabilité et réutilisabilité</li>
                    </ul>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="analyse-besoins-contrainte" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <BookOpen className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 2 — Analyse des besoins et contraintes mécaniques
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <article id="collaboration-reunions">
                      <h3 className="text-lg font-semibold mb-2">2.1 Collaboration et réunions stratégiques</h3>
                      <p>Avant de débuter la conception mécanique du convoyeur WasteFlow, plusieurs réunions stratégiques ont été organisées, réunissant les équipes électronique, informatique et mécanique. Ces échanges collaboratifs ont été essentiels pour garantir une vision commune et une compréhension approfondie des besoins et contraintes propres à chaque discipline.</p>
                      <p>Durant ces ateliers, chaque équipe a pu exposer ses attentes, ses exigences techniques, ainsi que les limitations liées à ses composants et méthodes. La mécanique, par exemple, a présenté les spécificités liées à l'impression 3D, la robustesse structurelle et la modularité du système. Les échanges ont également porté sur la compatibilité des interfaces physiques, la gestion des câblages, et les accès nécessaires pour la maintenance.</p>
                      <p>Ces réunions ont permis de mettre en lumière les points critiques d'intégration, d'anticiper les difficultés potentielles, et d'aligner les priorités pour le développement. Un cadre commun a été défini afin d'assurer la cohérence des choix techniques et faciliter la collaboration continue tout au long du projet.</p>
                      <p>Les principaux objectifs de ces échanges étaient :</p>
                      <ul className="list-disc pl-5 space-y-1">
                        <li>partager les attentes et contraintes spécifiques à chaque équipe,</li>
                        <li>identifier les interfaces critiques et points d'intégration,</li>
                        <li>définir un cadre commun de conception et validation,</li>
                        <li>prioriser les exigences fonctionnelles et techniques,</li>
                        <li>assurer une coordination globale et un suivi régulier du projet.</li>
                      </ul>
                      <p>Cette approche collaborative a posé les bases solides nécessaires à la réussite du développement du convoyeur WasteFlow, en garantissant que les solutions mécaniques soient pleinement compatibles avec les contraintes électroniques et informatiques.</p>
                    </article>
                    
                    <article id="analyse-fonctionnelle-graphique">
                      <h3 className="text-lg font-semibold mb-2">2.2 Analyse fonctionnelle graphique</h3>
                      <p>L'analyse fonctionnelle graphique permet de visualiser les interactions entre le système mécanique du convoyeur WasteFlow et son environnement, ainsi que les interfaces avec les autres disciplines. Le diagramme de pieuvre ci-dessous synthétise les principales fonctions contraintes et échanges, en mettant en évidence les flux d'énergie, d'information et de matière.</p>
                      <p>Ce diagramme facilite la compréhension globale des exigences mécaniques, tout en servant de base pour affiner la conception et identifier les points d'attention majeurs.</p>
                      <figure className="text-center mt-4">
                        <Image src="/assets/imgs/diagram.png" alt="Diagramme de pieuvre du convoyeur WasteFlow" width={600} height={400} className="max-w-full h-auto border p-2 mx-auto" />
                        <figcaption className="text-sm text-gray-600 mt-2">Diagramme de pieuvre simplifié illustrant les interactions principales du convoyeur WasteFlow.</figcaption>
                      </figure>
                    </article>
                    
                    <article id="contraintes-generales">
                      <h3 className="text-lg font-semibold mb-2">2.3 Contraintes générales</h3>
                      <p>À la lumière des échanges entre les équipes électronique, informatique et mécanique, il ressort plusieurs contraintes globales qui encadrent la conception du système. Ces contraintes concernent l'ensemble des sous-systèmes et assurent une cohérence optimale entre les différentes parties du projet.</p>
                      <p>Parmi les contraintes communes retenues, on trouve notamment :</p>
                      <ul className="list-disc pl-5 space-y-2">
                        <li>la nécessité d'une communication fluide et continue entre les équipes tout au long du projet</li>
                        <li>le respect des dimensions globales du convoyeur pour une installation optimale dans l'espace prévu</li>
                        <li>l'intégration harmonieuse des sous-systèmes (mécanique, électronique, informatique)</li>
                        <li>l'utilisation de composants standards et modulaires pour faciliter la maintenance et l'évolution du système</li>
                        <li>le respect des contraintes d'alimentation électrique pour assurer l'autonomie et la sécurité</li>
                      </ul>
                      <p>Ces contraintes ont guidé toutes les phases du projet, depuis la conception jusqu'à la réalisation, garantissant ainsi un convoyeur à la fois fonctionnel, fiable et adaptable.</p>
                    </article>
                    <article id="contraintes-mecaniques">
                      <h3 className="text-lg font-semibold mb-2">2.4 Contraintes mécaniques spécifiques</h3>
                      <p>Voici un tableau récapitulatif des principales contraintes mécaniques retenues, leur description et leur priorité dans la conception du convoyeur "WasteFlow".</p>
                      <div className="overflow-x-auto" suppressHydrationWarning={true}>
                        <table className="w-full text-sm border-collapse">
                          <thead>
                            <tr className="bg-primary text-primary-foreground">
                              <th className="p-2 border text-left">Contraintes</th>
                              <th className="p-2 border text-left">Description</th>
                              <th className="p-2 border text-center">Priorité</th>
                              <th className="p-2 border text-left">Remarques</th>
                            </tr>
                          </thead>
                          <tbody>
                            <tr className="bg-muted/50">
                                <td className="p-2 border">Dimensions</td>
                                <td className="p-2 border">Longueur totale de 650 mm et hauteur du tapis de 100 mm, contraintes d'encombrement</td>
                                <td className="p-2 border text-center">Élevée</td>
                                <td className="p-2 border">Respect strict pour intégration dans l'espace défini</td>
                            </tr>
                            <tr>
                                <td className="p-2 border">Compatibilité capteurs</td>
                                <td className="p-2 border">Supports réglables pour capteurs de couleur et laser intégrés à la structure</td>
                                <td className="p-2 border text-center">Moyenne</td>
                                <td className="p-2 border">Nécessite flexibilité pour ajustements lors des tests</td>
                            </tr>
                            <tr className="bg-muted/50">
                                <td className="p-2 border">Assemblage</td>
                                <td className="p-2 border">Démontage facile pour maintenance et modifications</td>
                                <td className="p-2 border text-center">Élevée</td>
                                <td className="p-2 border">Utilisation de vis M3/M4 et emboîtements adaptés</td>
                            </tr>
                            <tr>
                                <td className="p-2 border">Robustesse</td>
                                <td className="p-2 border">Résistance aux vibrations et chocs pendant le fonctionnement</td>
                                <td className="p-2 border text-center">Élevée</td>
                                <td className="p-2 border">Matériaux et épaisseurs adaptées, renforts locaux</td>
                            </tr>
                            <tr className="bg-muted/50">
                                <td className="p-2 border">Modularité</td>
                                <td className="p-2 border">Conception modulaire pour évolutions et réparations rapides</td>
                                <td className="p-2 border text-center">Moyenne</td>
                                <td className="p-2 border">Pièces découplées et connecteurs standards</td>
                            </tr>
                            <tr>
                                <td className="p-2 border">Tolérances impression 3D</td>
                                <td className="p-2 border">Jeu fonctionnel minimal de 0,3 mm sur pièces mobiles</td>
                                <td className="p-2 border text-center">Élevée</td>
                                <td className="p-2 border">Adaptation aux spécificités FDM et matériaux utilisés</td>
                            </tr>
                            <tr className="bg-muted/50">
                                <td className="p-2 border">Centre de masse</td>
                                <td className="p-2 border">Positionnement pour assurer stabilité et équilibre du convoyeur</td>
                                <td className="p-2 border text-center">Moyenne</td>
                                <td className="p-2 border">Influence directe sur le comportement dynamique</td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                    </article>
                    <article id="analyse-fonctionnelle-mecanique">
                      <h3 className="text-lg font-semibold mb-2">2.5 Analyse fonctionnelle mécanique</h3>
                      <p>L'analyse fonctionnelle mécanique vise à décomposer le système du convoyeur en fonctions principales et secondaires, afin d'identifier précisément les besoins mécaniques à satisfaire pour assurer le bon fonctionnement du système. Cette étape est cruciale pour orienter la conception et garantir la fiabilité de la structure.</p>
                      <p>Les fonctions principales du convoyeur mécanique sont :</p>
                      <ul className="list-disc pl-5 space-y-2 mb-6">
                        <li>supporter la bande transporteuse et assurer son guidage sans déformation</li>
                        <li>permettre la détection précise des déchets via une intégration stable des capteurs</li>
                        <li>faciliter le déplacement fluide et contrôlé des objets (cubes colorés) sur la surface du tapis</li>
                        <li>assurer la robustesse face aux vibrations et aux sollicitations mécaniques répétées</li>
                        <li>permettre un accès facile aux composants pour la maintenance et les ajustements</li>
                      </ul>
                      <p>En complément, les fonctions secondaires participent à la modularité, à la facilité d'assemblage et à la compatibilité avec les contraintes d'impression 3D et d'intégration électronique.</p>
                      <p>Cette analyse guidera la sélection des matériaux, le choix des procédés de fabrication, ainsi que la définition des jeux et tolérances.</p>
                      <figure className="text-center mt-4">
                        <Image src="/assets/imgs/diagramm.png" alt="Diagramme bête à cornes du convoyeur" width={600} height={400} className="max-w-full h-auto border p-2 mx-auto" />
                        <figcaption className="text-sm text-gray-600 mt-2">Diagramme fonctionnel « bête à cornes » simplifié du convoyeur WasteFlow.</figcaption>
                      </figure>
                      <p className="text-sm text-gray-700 mt-4">Ce diagramme simplifié illustre le convoyeur comme un système mécanique soumis à son environnement, aux contraintes spécifiques (dimensions, matériaux, intégration électronique) et aux besoins fonctionnels (déplacement, guidage, robustesse). Il aide à visualiser les interactions et les paramètres clés à prendre en compte lors de la conception.</p>
                    </article>
                    <article id="hypotheses-techniques" className="max-w-4xl">
                      <h3 className="text-lg font-semibold mb-4">2.6 Hypothèses techniques</h3>
                      
                      <h4 className="font-medium text-base mb-2">2.6.1 Environnement et conditions d'utilisation</h4>
                      <p className="mb-4">Le système est conçu pour un usage intérieur, mais doit fonctionner de manière fiable dans différents environnements lumineux et conditions ambiantes.<br />Pour assurer la stabilité des mesures du capteur couleur, celui-ci est fixé à une distance constante de 5 cm au-dessus de la bande transporteuse, conformément aux recommandations de l'équipe électronique.</p>
                      
                      <h4 className="font-medium text-base mb-2">2.6.2 Matériaux et composants mécaniques</h4>
                      <ul className="list-disc pl-5 space-y-2 mb-4">
                        <li><strong>Impression 3D PLA :</strong> Le choix du PLA pour la fabrication des pièces imprimées repose sur sa rigidité, sa stabilité dimensionnelle et sa facilité d'impression. Ce matériau est adapté pour des pièces structurelles et fonctionnelles sous faibles charges, avec un remplissage optimisé (50 %) pour un bon compromis solidité/légèreté.</li>
                        <li><strong>Composants standards :</strong> Certains éléments mécaniques comme les roulements 6202-2RS et les écrous M3/M4 ont été sélectionnés pour garantir la fiabilité des assemblages mobiles.</li>
                        <li><strong>Tapis :</strong> La bande transporteuse est conçue en matériau de chambre à air de moto, retenu pour sa résistance et flexibilité. Malgré son poids élevé, ce choix a été validé pour assurer une bonne adhérence et longévité. Les difficultés liées à ce matériau seront détaillées dans la section dédiée.</li>
                      </ul>
                      
                      <h4 className="font-medium text-base mb-2">2.6.3 Moyens de fabrication et optimisation</h4>
                      <p className="mb-2">Les pièces seront imprimées sur des imprimantes 3D BambuLab et Prusa, compatibles avec des volumes jusqu'à 220 x 220 x 250 mm.<br />L'optimisation porte sur :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li>le découpage en sous-ensembles assemblables facilement,</li>
                        <li>l'orientation des pièces pour minimiser les supports et maximiser la résistance,</li>
                        <li>la réduction de la consommation de matière sans compromettre la solidité.</li>
                      </ul>
                      
                      <h4 className="font-medium text-base mb-2">2.6.4 Durabilité et usage industriel</h4>
                      <p className="mb-4">Le convoyeur est conçu pour un usage industriel, avec des matériaux et assemblages offrant une durabilité adaptée à un fonctionnement continu.<br />Le PLA imprimé, bien conçu, assure une robustesse satisfaisante, mais la modularité permet aussi un remplacement facile des pièces en cas d'usure.</p>
                      
                      <h4 className="font-medium text-base mb-2">2.6.5 Mouvements et charges</h4>
                      <p className="mb-4">Les éléments mécaniques incluent des mouvements rotatifs (roulements 6202-2RS, tambours, arbres) et linéaires (glissières pour la tension de la bande).<br />Les charges exercées restent faibles, liées au poids des cubes en PLA (30 mm, creux, remplissage 50 %) et à la force moteur DC. La vitesse n'est pas encore définie précisément, la conception intègre une marge de sécurité.</p>
                      
                      <h4 className="font-medium text-base mb-2">2.6.6 Interface avec l'électronique centrale</h4>
                      <p className="mb-4">La conception mécanique intègre la nécessité de recevoir les composants électroniques centraux (Arduino Nano, ESP32) dans des logements adaptés.<br />Le dimensionnement précis de ces logements attend les spécifications finales de l'équipe électronique, pour garantir intégration, accessibilité et protection optimales.</p>
                    </article>
                    <article id="specifications-minimales">
                      <h3 className="text-lg font-semibold mb-2">2.7 Rappel des spécifications minimales</h3>
                      <p>Cette liste synthétise les exigences dimensionnelles et techniques essentielles à respecter pour garantir la réussite du projet :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li>longueur totale du convoyeur : 650 mm</li>
                        <li>hauteur du tapis par rapport au sol : 100 mm</li>
                        <li>dimensions des déchets : cubes de 30 mm de côté</li>
                        <li>utilisation d'un microcontrôleur Arduino Nano ou ATmega328P</li>
                        <li>alimentation par bloc de batteries Lithium</li>
                        <li>capteur couleur placé à 5 cm du tapis transporteur</li>
                        <li>utilisation du PLA imprimable en 3D pour la majorité des pièces mécaniques</li>
                        <li>roulements 6202-2RS et écrous M3/M4 pour les assemblages mécaniques</li>
                        <li>structure modulaire et robuste pour faciliter maintenance et évolutions</li>
                      </ul>
                      <p><strong>Cette synthèse marque la transition vers la phase de conception mécanique</strong>, où chaque exigence sera traduite en modélisations précises.</p>
                    </article>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="conception-mecanique" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Cog className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 3 — Conception mécanique
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <p>À partir des analyses, contraintes et spécifications minimales précédemment définies, la phase de conception mécanique débute ici. L'objectif est de traduire les exigences en modèles 3D précis, fonctionnels et optimisés, compatibles avec les matériaux et techniques retenues.</p>
                    <p>Cette étape inclut la modélisation des pièces principales, la définition des assemblages, ainsi que la prise en compte des aspects d'impression 3D, robustesse et modularité. Chaque choix sera justifié en fonction des contraintes mécaniques et des objectifs du projet.</p>
                    <p><strong>Nous détaillerons successivement :</strong></p>
                    <ul className="list-disc pl-5 space-y-1 mb-6">
                      <li>la conception des pièces et sous-ensembles principaux</li>
                      <li>les choix techniques pour les assemblages et fixations</li>
                      <li>les stratégies pour assurer robustesse, modularité et facilité de maintenance</li>
                      <li>les contraintes d'impression 3D et optimisation des matériaux</li>
                    </ul>
                    
                    <article id="architecture-globale">
                      <h3 className="text-lg font-semibold mb-4">3.1 Aperçu global de l'architecture mécanique</h3>
                      <p>Le tableau ci-dessous présente les différents sous-ensembles mécaniques, leur rôle principal et les points clés de conception associés.</p>
                      <div className="overflow-x-auto mt-4" suppressHydrationWarning={true}>
                        <table className="w-full text-sm border-collapse">
                          <thead>
                            <tr className="bg-muted">
                              <th className="border p-2 text-left">Sous-ensemble</th>
                              <th className="border p-2 text-left">Rôle principal</th>
                              <th className="border p-2 text-left">Points clés de conception</th>
                            </tr>
                          </thead>
                          <tbody>
                            <tr>
                              <td className="border p-2">Châssis principal</td>
                              <td className="border p-2">Supporter et stabiliser l'ensemble du système</td>
                              <td className="border p-2">Robustesse, rigidité, intégration des autres composants</td>
                            </tr>
                            <tr>
                              <td className="border p-2">Bande transporteuse</td>
                              <td className="border p-2">Transporter les cubes entre les différents points du tri</td>
                              <td className="border p-2">Matériau résistant, poids optimisé, bonne adhérence</td>
                            </tr>
                            <tr>
                              <td className="border p-2">Système de motorisation</td>
                              <td className="border p-2">Assurer la rotation fluide de la bande</td>
                              <td className="border p-2">Roulements adaptés (6202-2RS), arbre aligné, moteur DC</td>
                            </tr>
                            <tr>
                              <td className="border p-2">Glissières et tension</td>
                              <td className="border p-2">Régler et maintenir la tension optimale de la bande</td>
                              <td className="border p-2">Système ajustable, tolérance d'usure, entretien facile</td>
                            </tr>
                            <tr>
                              <td className="border p-2">Supports capteurs</td>
                              <td className="border p-2">Maintenir les capteurs en position idéale</td>
                              <td className="border p-2">Positionnement précis (5 cm de la bande), protection</td>
                            </tr>
                            <tr>
                              <td className="border p-2">Fixations et assemblages</td>
                              <td className="border p-2">Relier les composants de manière modulaire</td>
                              <td className="border p-2">Vis M3/M4, démontage rapide, résistance mécanique</td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                      <p className="mt-4">Cette architecture a été pensée pour favoriser la maintenance, la modularité et la compatibilité avec l'impression 3D, tout en assurant une intégration harmonieuse avec l'électronique et les capteurs.</p>
                    </article>
                    <article id="composants-standards">
                      <h3 className="text-lg font-semibold mb-4">3.2 Composants standards utilisés</h3>
                      <p>Certains éléments du convoyeur ont été conçus en intégrant des composants standards du commerce. Leur utilisation permet de garantir fiabilité, compatibilité et gain de temps en conception, tout en facilitant le remplacement en cas de maintenance. Les modèles 3D ont été intégrés à la conception mécanique lorsque nécessaire.</p>
                      <div className="overflow-x-auto mt-4" suppressHydrationWarning={true}>
                        <table className="w-full text-sm border-collapse">
                          <thead>
                            <tr className="bg-muted">
                              <th className="border p-2 text-left">Composant</th>
                              <th className="border p-2 text-left">Description / Rôle</th>
                              <th className="border p-2 text-left">Lien modèle 3D (.zip)</th>
                            </tr>
                          </thead>
                          <tbody>
                            <tr>
                              <td className="border p-2">Roulement 6202-2RS</td>
                              <td className="border p-2">Roulement à billes étanche utilisé pour assurer la rotation fluide des tambours du convoyeur.</td>
                              <td className="border p-2"><a href="/assets/roulement-6202.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                            <tr>
                              <td className="border p-2">Moteur DC</td>
                              <td className="border p-2">Moteur électrique à courant continu pour entraîner la bande transporteuse.</td>
                              <td className="border p-2"><a href="/assets/motor_DC.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                            <tr>
                              <td className="border p-2">Capteur KY-008 (Laser)</td>
                              <td className="border p-2">Module laser utilisé pour la détection de passage et synchronisation avec la mécanique.</td>
                              <td className="border p-2"><a href="/assets/ky008.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                            <tr>
                              <td className="border p-2">Capteur de couleur TCS34725</td>
                              <td className="border p-2">Capteur RGB haute précision utilisé pour identifier les couleurs des cubes avec correction de lumière.</td>
                              <td className="border p-2"><a href="/assets/tcs34725.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                      <p className="mt-3 text-sm italic">Tous les modèles 3D ont été recherchés principalement sur <a href="https://grabcad.com" target="_blank" className="text-blue-600 hover:underline font-bold">GrabCAD – la mine d'or des ingénieurs !</a></p>
                      
                      <h4 className="mt-6 font-medium">Prévisualisation des composants</h4>
                      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mt-4">
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/assets/imgs/moteur_DC.png" alt="Moteur DC" width={150} height={220} className="mx-auto" />
                          <figcaption className="text-xs italic font-bold mt-2">Moteur DC</figcaption>
                        </figure>
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/assets/imgs/ky008_laser.png" alt="Capteur KY-008" width={150} height={220} className="mx-auto" />
                          <figcaption className="text-xs italic font-bold mt-2">Capteur KY-008</figcaption>
                        </figure>
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/assets/imgs/roulement-6202.png" alt="Roulement 6202-2RS" width={150} height={220} className="mx-auto" />
                          <figcaption className="text-xs italic font-bold mt-2">Roulement 6202-2RS</figcaption>
                        </figure>
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/assets/imgs/tcs34725.png" alt="Capteur TCS34725" width={150} height={220} className="mx-auto" />
                          <figcaption className="text-xs italic font-bold mt-2">Capteur de couleur TCS34725</figcaption>
                        </figure>
                      </div>
                    </article>
                    <article id="modelisation-3d-preliminaire">
                      <h3 className="text-lg font-semibold mb-4">3.3 Modélisation 3D préliminaire</h3>
                      <p>La modélisation 3D est une étape clé dans la conception mécanique du convoyeur. Elle permet de visualiser les pièces et assemblages, de vérifier la faisabilité des solutions retenues, et de préparer les fichiers nécessaires à l'impression 3D. Au cours du projet, plusieurs itérations de la modélisation ont été réalisées pour améliorer la robustesse, la modularité et la compatibilité avec les contraintes mécaniques et électroniques. Chaque étape correspond à une version de conception, dont les caractéristiques évoluent pour répondre aux exigences du projet.</p>
                      
                      <div className="space-y-12">
                        {/* VERSION 1 */}
                        <div id="modele-v1-detail">
                          <h4 className="text-base font-semibold mb-4">3.3.1 Première itération — Version initiale avec système de poulie</h4>
                          <p className="mb-4">La première itération visait à obtenir rapidement une représentation tangible du convoyeur en 3D. Le modèle présentait un convoyeur réalisé en un bloc unique intégrant tambours, tapis, arbre, un moteur et un système de poulie. Deux supports pour capteurs (couleur et laser) ont été ajoutés pour visualiser l'implantation des capteurs dans l'espace. Cette étape permettait essentiellement de valider l'implantation générale et les proportions, et non de produire une pièce directement imprimable.</p>
                          
                          <p className="mb-4">Les composants modélisés pour cette version sont, de manière synthétique :</p>
                          <ul className="list-disc pl-5 space-y-1 mb-6">
                            <li>le bâti du convoyeur (en un bloc),</li>
                            <li>le support du capteur couleur,</li>
                            <li>le support du capteur laser,</li>
                            <li>le système de poulie,</li>
                            <li>les roulements,</li>
                            <li>le tambour et</li>
                            <li>l'arbre</li>
                          </ul>
                          
                          <h5 className="font-medium mb-3">Illustrations et masses</h5>
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                            <figure className="text-center">
                              <Image src="/assets/imgs/part0-convoyeur_v0.png" alt="Bâti convoyeur v0" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Bâti du convoyeur — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/part0-conv_v0-mass-properties.png" alt="Bâti convoyeur v0 masse" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Bâti du convoyeur — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-6">
                            <figure className="text-center">
                              <Image src="/assets/imgs/support-tcs34725-conv_v0.png" alt="Support capteur couleur v0" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur couleur — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/support-tcs34725-conv_v0-mass-properties.png" alt="Support capteur couleur v0 masse" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur couleur — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-6">
                            <figure className="text-center">
                              <Image src="/assets/imgs/support-ky008-conv_v0.png" alt="Support capteur laser v0" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur laser — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/support-ky008-conv_v0-mass-properties.png" alt="Support capteur laser v0 masse" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur laser — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-6">
                            <figure className="text-center">
                              <Image src="/assets/imgs/poulie-conv_v0.png" alt="Système poulie v0" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Système de poulie — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/poulie-conv_v0-mass-properties.png" alt="Système poulie v0 masse" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Système de poulie — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mt-6">
                            <figure className="text-center">
                              <Image src="/assets/imgs/roulement-conv_v0.png" alt="Roulement v0" width={300} height={350} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Roulement — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/arbre-conv_v0.png" alt="Arbre v0" width={300} height={350} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Arbre — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/tambour-conv_v0.png" alt="Tambour v0" width={300} height={350} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Tambour — Vue 3D</figcaption>
                            </figure>
                          </div>
                          
                          <h5 className="font-medium mt-8 mb-3">Illustrations de l'assemblage au complet et de sa masse</h5>
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                            <figure className="text-center">
                              <Image src="/assets/imgs/convoyeur_v0.png" alt="Convoyeur v0" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v0 — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/assets/imgs/conv_v0-mass-properties.png" alt="Convoyeur v0 masse" width={400} height={400} className="rounded-md border mx-auto"/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v0 — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                        </div>
                      </div>
                    </article>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="details-techniques-assemblage" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                   <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <HardHat className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 4 — Détails techniques et assemblage
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                      {/* ... (contenu de la section 4) ... */}
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="fabrication" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                   <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Cog className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 5 — Fabrication
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                      {/* ... (contenu de la section 5, incluant le tableau BOM et toutes les images de slicing) ... */}
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="difficultes-solutions" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                   <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <AlertTriangle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 6 — Difficultés rencontrées et solutions
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                      {/* ... (contenu de la section 6) ... */}
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card id="conclusion" className="border border-gray-200 dark:border-gray-700 scroll-mt-24">
                   <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      Section 7 — Conclusion et perspectives
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                      {/* ... (contenu de la section 7) ... */}
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