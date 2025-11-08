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
    <Image 
      src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_ifri.png" 
      alt="Convoyeur WasteFlow" 
      fill 
      className="absolute inset-0 object-cover z-0" 
      unoptimized 
    />
    <div className="absolute inset-0 bg-black opacity-50 z-10"></div>
    <div className="relative z-20">
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
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/diagram.png" alt="Diagramme de pieuvre du convoyeur WasteFlow" width={600} height={400} className="max-w-full h-auto border p-2 mx-auto" unoptimized />
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
                        <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/diagramm.png" alt="Diagramme bête à cornes du convoyeur" width={600} height={400} className="max-w-full h-auto border p-2 mx-auto" unoptimized />
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
                              <td className="border p-2"><a href="/Documentation/test-final/assets/zips/roulement-6202.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                            <tr>
                              <td className="border p-2">Moteur DC</td>
                              <td className="border p-2">Moteur électrique à courant continu pour entraîner la bande transporteuse.</td>
                              <td className="border p-2"><a href="/Documentation/test-final/assets/zips/motor_DC.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                            <tr>
                              <td className="border p-2">Capteur KY-008 (Laser)</td>
                              <td className="border p-2">Module laser utilisé pour la détection de passage et synchronisation avec la mécanique.</td>
                              <td className="border p-2"><a href="/Documentation/test-final/assets/zips/ky008.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                            <tr>
                              <td className="border p-2">Capteur de couleur TCS34725</td>
                              <td className="border p-2">Capteur RGB haute précision utilisé pour identifier les couleurs des cubes avec correction de lumière.</td>
                              <td className="border p-2"><a href="/Documentation/test-final/assets/zips/tcs34725.zip" className="text-blue-600 hover:underline">Télécharger</a></td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                      <p className="mt-3 text-sm italic">Tous les modèles 3D ont été recherchés principalement sur <a href="https://grabcad.com" target="_blank" className="text-blue-600 hover:underline font-bold">GrabCAD – la mine d'or des ingénieurs !</a></p>
                      
                      <h4 className="mt-6 font-medium">Prévisualisation des composants</h4>
                      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mt-4">
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/moteur_DC.png" alt="Moteur DC" width={150} height={220} className="mx-auto" unoptimized />
                          <figcaption className="text-xs italic font-bold mt-2">Moteur DC</figcaption>
                        </figure>
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/ky008_laser.png" alt="Capteur KY-008" width={150} height={220} className="mx-auto" unoptimized />
                          <figcaption className="text-xs italic font-bold mt-2">Capteur KY-008</figcaption>
                        </figure>
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/roulement-6202.png" alt="Roulement 6202-2RS" width={150} height={220} className="mx-auto" unoptimized />
                          <figcaption className="text-xs italic font-bold mt-2">Roulement 6202-2RS</figcaption>
                        </figure>
                        <figure className="text-center border rounded-lg p-2">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/tcs34725.png" alt="Capteur TCS34725" width={150} height={220} className="mx-auto" unoptimized />
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
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/part0-convoyeur_v0.png" alt="Bâti convoyeur v0" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Bâti du convoyeur — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/part0-conv_v0-mass-properties.png" alt="Bâti convoyeur v0 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Bâti du convoyeur — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/support-tcs34725-conv_v0.png" alt="Support capteur couleur v0" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur couleur — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/support-tcs34725-conv_v0-mass-properties.png" alt="Support capteur couleur v0 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur couleur — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/support-ky008-conv_v0.png" alt="Support capteur laser v0" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur laser — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/support-ky008-conv_v0-mass-properties.png" alt="Support capteur laser v0 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Support capteur laser — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mt-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/poulie-conv_v0.png" alt="Système poulie v0" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Système de poulie — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/poulie-conv_v0-mass-properties.png" alt="Système poulie v0 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Système de poulie — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mt-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/roulement-conv_v0.png" alt="Roulement v0" width={300} height={350} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Roulement — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/arbre-conv_v0.png" alt="Arbre v0" width={300} height={350} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Arbre — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/tambour-conv_v0.png" alt="Tambour v0" width={300} height={350} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Tambour — Vue 3D</figcaption>
                            </figure>
                          </div>
                          
                          <h5 className="font-medium mt-8 mb-3">Illustrations de l'assemblage au complet et de sa masse</h5>
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v0.png" alt="Convoyeur v0" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v0 — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv_v0-mass-properties.png" alt="Convoyeur v0 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v0 — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                        </div>

                        {/* VERSION 2 */}
                        <div id="modele-v2-detail" className="mt-12">
                          <h4 className="text-base font-semibold mb-4">3.3.2 Deuxième itération — Découpage en deux et optimisation des tambours</h4>
                          <p className="mb-4">Suite aux enseignements tirés de la première itération, la deuxième version a opéré un changement structurel fort : le convoyeur a été découpé en deux sous-ensembles pour tenir compte des limites d'impression (volume utile ≈ 250 × 250 mm) et faciliter la fabrication. Le découpage vise aussi à améliorer la manutention et la maintenance. Les roulements, qui avaient été précédemment seulement modélisés « pour montrer », ont été replacés dans des paliers en U usinés dans le bâti afin de permettre le passage du tambour ; le tambour a lui-même été remanié pour intégrer l'arbre et des cavités d'adhérence destinées à améliorer l'engagement du tapis.</p>
                          
                          <p className="mb-4">Le système de poulie a été supprimé ; le moteur a été rapproché vers l'un des bâti — dorénavant appelé « bâti moteur » — et monté sur un support distinct. Ce support est traité comme un composant séparé, fixé sur le bâti moteur pour concentrer les efforts et limiter les sollicitations sur la structure principale. Parallèlement, des enlèvements de matière ont été effectués sur les poteaux et certaines faces pour réduire la masse globale. Ces réductions restent partielles et pourront être optimisées ultérieurement sans compromettre la rigidité.</p>
                          
                          <p className="mb-4">Les composants modélisés pour cette version sont, de manière synthétique :</p>
                          <ul className="list-disc pl-5 space-y-1 mb-6">
                            <li>le bâti moteur,</li>
                            <li>le bâti non moteur,</li>
                            <li>le support du moteur DC,</li>
                            <li>le tambour moteur,</li>
                            <li>le tambour non moteur</li>
                          </ul>
                          
                          <h5 className="font-medium mb-3">Illustrations et masses</h5>
                          <div className="space-y-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-decoupe-conv_v1-mass-properties.png" alt="Bâti découpé v1" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Bâti découpé du convoyeur — Vue 3D + Propriétés de masse</figcaption>
                            </figure>
                            
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/tambour-non-moteur-conv_v1-mass-properties.png" alt="Tambour non moteur v1" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Tambour non moteur — Vue 3D + Propriétés de masse</figcaption>
                            </figure>
                            
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/tambour-moteur-conv_v1-mass-properties.png" alt="Tambour moteur v1" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Tambour moteur — Vue 3D + Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <h5 className="font-medium mt-8 mb-3">Illustrations de l'assemblage au complet et de sa masse</h5>
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v1.png" alt="Convoyeur v1" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v1 — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv_v1-mass-properties.png" alt="Convoyeur v1 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v1 — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <h5 className="font-medium mt-8 mb-3">Tableau récapitulatif de l'état du convoyeur</h5>
                          <p className="mb-4">Le tableau récapitulatif ci-dessous présente l'état des composants dans cette deuxième itération.</p>
                          <div className="overflow-x-auto" suppressHydrationWarning={true}>
                            <table className="w-full text-sm border-collapse">
                              <thead>
                                <tr className="bg-muted">
                                  <th className="border p-2 text-left">Composants</th>
                                  <th className="border p-2 text-center">Masse (en grammes)</th>
                                  <th className="border p-2 text-center">Prêt pour impression</th>
                                  <th className="border p-2 text-left">Commentaires</th>
                                </tr>
                              </thead>
                              <tbody>
                                <tr>
                                  <td className="border p-2">Bâti découpé du convoyeur + support moteur</td>
                                  <td className="border p-2 text-right">3346</td>
                                  <td className="border p-2 text-center">Non (à ajuster)</td>
                                  <td className="border p-2">Moitié du convoyeur conçue pour l'impression. Paliers en U présents mais retention non finalisée ; prévoir points de fixation/goupilles de centrage pour l'assemblage.</td>
                                </tr>
                                <tr>
                                  <td className="border p-2">Support capteur couleur</td>
                                  <td className="border p-2 text-right">41</td>
                                  <td className="border p-2 text-center">Non</td>
                                  <td className="border p-2">Position envisagée ; nécessite ajout de cache anti-lumière ambiante et fixation modulable pour ajustement à 5 cm.</td>
                                </tr>
                                <tr>
                                  <td className="border p-2">Support capteur laser</td>
                                  <td className="border p-2 text-right">11</td>
                                  <td className="border p-2 text-center">Non</td>
                                  <td className="border p-2">Similaire au support couleur ; prévoir positionnement fin et protection contre vibrations.</td>
                                </tr>
                                <tr>
                                  <td className="border p-2">Tambour non moteur</td>
                                  <td className="border p-2 text-right">441</td>
                                  <td className="border p-2 text-center">Non (ajustements d'alésage)</td>
                                  <td className="border p-2">Conception consolidée pour améliorer adhérence du tapis. Prévoir manchon d'ajustement ou arbre métallique pour montage réel.</td>
                                </tr>
                                <tr>
                                  <td className="border p-2">Tambour moteur</td>
                                  <td className="border p-2 text-right">453</td>
                                  <td className="border p-2 text-center">Non (ajustements d'alésage)</td>
                                  <td className="border p-2">Conception consolidée pour améliorer adhérence du tapis. Prévoir manchon d'ajustement ou arbre métallique pour montage réel.</td>
                                </tr>
                                <tr className="bg-muted font-semibold">
                                  <td className="border p-2">Total / Remarque</td>
                                  <td className="border p-2 text-right">4785.83g</td>
                                  <td className="border p-2 text-center">—</td>
                                  <td className="border p-2">Version préparée pour tests d'assemblage. Nécessite finalisation des sièges de roulement, retenues axiales, jonction entre demi-bâti et renforts locaux.</td>
                                </tr>
                              </tbody>
                            </table>
                          </div>
                        </div>

                        {/* VERSION 3 - FINALE */}
                        <div id="modele-v3-final" className="mt-12">
                          <h4 className="text-base font-semibold mb-4">3.3.3 Troisième itération — Version finale préparée pour impression</h4>
                          <p className="mb-4">La dernière itération reflète le travail de recherche, d'essais et d'optimisation effectué après les premières versions. Les roulements ont été correctement montés grâce à l'ajout de couvercles à lèvres (droite / gauche) et d'épaulements sur les tambours pour assurer une retenue axiale fiable. Le couvercle gauche est volontairement légèrement plus petit que le droit afin de laisser l'espace nécessaire au montage des roulements. Le support du moteur a été fixé sur le bâti moteur pour améliorer la rigidité et la tenue mécanique du montage.</p>
                          
                          <p className="mb-4">Des chanfreins et congés ont été ajoutés sur les arêtes saillantes pour réduire les risques de blessure et diminuer localement la matière. Un système de tension a été conçu au niveau du bâti non-moteur : il s'agit d'une glissière réglable maintenue par vis et écrous qui permet d'ajuster précisément la pré-tension du tapis. Les couvercles des paliers sont fixés par visserie (M3/M4) avec inserts prévus dans les pièces imprimées.</p>
                          
                          <h5 className="font-medium mb-3">Composants modélisés (liste exhaustive)</h5>
                          <p className="mb-4">Les pièces suivantes ont été modélisées pour cette version finale et feront l'objet d'illustrations individuelles (vue 3D + capture Mass Properties) ainsi que d'un fichier téléchargeable (STEP / STL) :</p>
                          <ul className="list-disc pl-5 space-y-1 mb-6">
                            <li>2 tambours (moteur / non-moteur)</li>
                            <li>2 couvercles à lèvres (gauche / droite)</li>
                            <li>support - capteur de couleur</li>
                            <li>support - capteur laser</li>
                            <li>bâti moteur - partie 1</li>
                            <li>bâti moteur - partie 2</li>
                            <li>paliers porte-roulements (coulisseau)</li>
                            <li>bâti non-moteur - partie 1</li>
                            <li>bâti non-moteur - partie 2</li>
                            <li>jointure - partie 1</li>
                            <li>jointure - partie 2</li>
                            <li>cube (élément de test / déchet)</li>
                          </ul>
                          
                          <h5 className="font-medium mb-3">Illustrations et masses</h5>
                          <div className="space-y-8">
                            {/* Tambours et supports */}
                            <div className="space-y-6">
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/tambour-non-moteur-conv_v3-mass-properties.png" alt="Tambour non moteur v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Tambour non moteur — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                              
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/tambour-moteur-conv_v3-mass-properties.png" alt="Tambour moteur v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Tambour moteur — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                              
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/support-ky008-conv_v3-mass-properties.png" alt="Support capteur laser v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Support capteur laser — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                              
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/support-tcs34725-conv_v3-mass-properties.png" alt="Support capteur couleur v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Support capteur couleur — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                              
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/couvercle-a-levres-conv_v3-mass-properties.png" alt="Couvercle à lèvres v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Couvercle à lèvres — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                              
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/porte-roulemement-non-moteur-conv_v3-mass-properties.png" alt="Coulisseau porte roulement v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Coulisseau porte roulement — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                            </div>
                            
                            {/* Partie moteur */}
                            <div>
                              <h6 className="font-medium mb-4">--- Convoyeur : Partie moteur ---</h6>
                              <div className="space-y-6">
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-moteur-1-conv_v3-mass-properties.png" alt="Bâti moteur partie 1 v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bâti moteur_partie1 — Vue 3D + Propriétés de masse</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-moteur-2-conv_v3-mass-properties.png" alt="Bâti moteur partie 2 v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bâti moteur_partie2 — Vue 3D + Propriétés de masse</figcaption>
                                </figure>
                              </div>
                            </div>
                            
                            {/* Partie non moteur */}
                            <div>
                              <h6 className="font-medium mb-4">--- Convoyeur : Partie non moteur ---</h6>
                              <div className="space-y-6">
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-non-moteur-1-conv_v3-mass-properties.png" alt="Bâti non moteur partie 1 v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bâti non moteur_partie1 — Vue 3D + Propriétés de masse</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-non-moteur-2-conv_v3-mass-properties.png" alt="Bâti non moteur partie 2 v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bâti non moteur_partie2 — Vue 3D + Propriétés de masse</figcaption>
                                </figure>
                              </div>
                            </div>
                            
                            {/* Jointure */}
                            <div>
                              <h6 className="font-medium mb-4">--- Convoyeur : Jointure ---</h6>
                              <div className="space-y-6">
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/jointure-1-conv_v3-mass-properties.png" alt="Jointure partie 1 v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Jointure_partie1 — Vue 3D + Propriétés de masse</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/jointure-2-conv_v3-mass-properties.png" alt="Jointure partie 2 v3" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Jointure_partie2 — Vue 3D + Propriétés de masse</figcaption>
                                </figure>
                              </div>
                            </div>
                            
                            {/* Convoyeur en 4 parties */}
                            <div>
                              <h6 className="font-medium mb-4">--- Convoyeur : en 4 parties ---</h6>
                              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-moteur-conv_v3.png" alt="Bâti moteur v3" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bâti moteur — Vue 3D</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bâti-non-moteur-conv_v3.png" alt="Bâti non moteur v3" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bâti non moteur — Vue 3D</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/jointure-conv_v3.png" alt="Jointure v3" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Jointure — Vue 3D</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/bande-transporteuse.png" alt="Bande transporteuse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Bande transporteuse — Vue 3D</figcaption>
                                </figure>
                              </div>
                            </div>
                            
                            {/* Cube 30x30mm */}
                            <div>
                              <h6 className="font-medium mb-4">--- Convoyeur : Cube 30*30mm ---</h6>
                              <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/cube-decoupe.png" alt="Cube découpé profil" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Cube découpé de profil — Vue 3D</figcaption>
                                </figure>
                                
                                <figure className="text-center">
                                  <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/cube-decoupe-0.png" alt="Cube découpé dessus" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                                  <figcaption className="text-xs text-gray-600 mt-2">Cube découpé de dessus — Vue 3D</figcaption>
                                </figure>
                              </div>
                              
                              <figure className="text-center">
                                <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/cube-mass-properties.png" alt="Cube propriétés masse" width={800} height={400} className="rounded-md border mx-auto" unoptimized/>
                                <figcaption className="text-xs text-gray-600 mt-2">Cube — Vue 3D + Propriétés de masse</figcaption>
                              </figure>
                            </div>
                          </div>
                          
                          <h5 className="font-medium mt-8 mb-3">Illustrations de l'assemblage au complet et de sa masse</h5>
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v3.png" alt="Convoyeur v3" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v3 — Vue 3D</figcaption>
                            </figure>
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv_v3-mass-properties.png" alt="Convoyeur v3 masse" width={400} height={400} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v3 — Propriétés de masse</figcaption>
                            </figure>
                          </div>
                          
                          <h5 className="font-medium mt-8 mb-3">Convoyeur - Vue 3D</h5>
                          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v3-dessus.png" alt="Convoyeur v3 dessus" width={400} height={200} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v3 — Vue de dessus</figcaption>
                            </figure>
                            
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v3-face.png" alt="Convoyeur v3 face" width={400} height={200} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v3 — Vue de face</figcaption>
                            </figure>
                            
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v3-droite.png" alt="Convoyeur v3 droite" width={400} height={200} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v3 — Vue de droite</figcaption>
                            </figure>
                            
                            <figure className="text-center">
                              <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/convoyeur_v3-dessous.png" alt="Convoyeur v3 dessous" width={400} height={200} className="rounded-md border mx-auto" unoptimized/>
                              <figcaption className="text-xs text-gray-600 mt-2">Convoyeur_v3 — Vue de dessous</figcaption>
                            </figure>
                          </div>
                          
                          <p className="mt-6 mb-4">La phase de conception 3D est close : nous disposons d'une version finale découpée, modélisée et prête à l'export pour impression. On passe maintenant à la phase technique d'assemblage et de fabrication.</p>
                          
                          <div className="bg-blue-50 dark:bg-blue-900/20 p-4 rounded-lg">
                            <h6 className="font-medium mb-2">Vous pouvez maintenant télécharger le zip de l'assemblage au complet. Il contient toutes les pièces modélisées et l'assemblage lui même.</h6>
                            <p className="text-sm italic">Pour télécharger, cliquez <a href="/Documentation/test-final/assets/zips/convoyeur.zip" className="text-blue-600 hover:underline">ici</a></p>
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
                  <CardContent className="p-6 space-y-6">
                    <p>Cette partie décrit les aspects concrets de fabrication et d'assemblage du convoyeur. Après la phase de conception, chaque composant doit maintenant être documenté pour l'impression, la préparation et le montage. L'objectif est d'assurer que toute personne puisse reproduire le système à partir des fichiers fournis et des indications techniques.</p>
                    
                    <article id="pieces-imprimees-3d">
                      <h3 className="text-lg font-semibold mb-4">4.1 Pièces imprimées en 3D</h3>
                      <p className="mb-4">Les éléments principaux ont été découpés et optimisés pour entrer dans le volume d'impression disponible (≈ 250 × 220 × 220 mm). Chaque pièce imprimée – qu'il s'agisse des tambours moteur et non-moteur, des becs à lèvres, des supports de capteurs ou des parties de bâti – sera présentée avec une vue 3D, une capture de sa masse issue de la CAO, et un lien de téléchargement aux formats STEP et STL.</p>
                      
                      <p className="mb-4">Les matériaux et formes ont été choisis pour allier solidité et facilité d'impression. Le PLA est privilégié pour sa stabilité dimensionnelle et sa simplicité de mise en œuvre. Les zones soumises à contraintes, comme les fixations moteur ou sièges de roulements, ont été renforcées par des épaulements et des congés. Les logements pour vis M3/M4 et inserts chauffants ont été intégrés dès la modélisation.</p>
                      
                      <div className="bg-teal-50 dark:bg-teal-900/20 border-l-4 border-teal-600 p-4 mb-4">
                        <h4 className="font-semibold text-teal-800 dark:text-teal-200 mb-2">Paramètres d'impression conseillés</h4>
                        <ul className="text-sm text-teal-700 dark:text-teal-300 space-y-1">
                          <li>Hauteur de couche : <strong>0,20 mm</strong></li>
                          <li>Remplissage : <strong>20 à 30 %</strong> (structurelles : 30 % / esthétiques : 15–20 %)</li>
                          <li>Épaisseur paroi : <strong>≥ 2 mm</strong> (2–3 périmètres)</li>
                          <li>Supports : uniquement pour porte-à-faux &gt; 45°</li>
                          <li>Jeu fonctionnel : <strong>≥ 0,3 mm</strong> pour pièces mobiles</li>
                          <li>Prévoir bossages et perçages adaptés pour inserts M3/M4</li>
                        </ul>
                      </div>
                    </article>
                    
                    <article id="composants-standards">
                      <h3 className="text-lg font-semibold mb-4">4.2 Composants standards</h3>
                      <p className="mb-4">Certains éléments essentiels du convoyeur ne sont pas imprimés mais achetés, afin de garantir fiabilité, précision et durabilité. Le roulement <strong>6202-2RS</strong>, par exemple, a été retenu car il est facilement disponible sur le marché, conforme aux normes industrielles et difficilement fabricable par impression 3D dans le cadre de notre projet.</p>
                      
                      <div className="overflow-x-auto" suppressHydrationWarning={true}>
                        <table className="w-full text-sm border-collapse">
                          <thead>
                            <tr className="bg-muted">
                              <th className="border p-2 text-left">Référence</th>
                              <th className="border p-2 text-left">Désignation</th>
                              <th className="border p-2 text-center">Quantité</th>
                              <th className="border p-2 text-left">Spécifications techniques</th>
                              <th className="border p-2 text-left">Justification</th>
                            </tr>
                          </thead>
                          <tbody>
                            <tr>
                              <td className="border p-2">STD-001</td>
                              <td className="border p-2">Roulement 6202-2RS</td>
                              <td className="border p-2 text-center">4</td>
                              <td className="border p-2">Norme ISO, Ø intérieur 15 mm, Ø extérieur 35 mm, largeur 11 mm</td>
                              <td className="border p-2">Disponible, conforme aux normes, impossible à fabriquer dans les délais</td>
                            </tr>
                            <tr>
                              <td className="border p-2">STD-002</td>
                              <td className="border p-2">Vis M3/M4 + écrous</td>
                              <td className="border p-2 text-center">20</td>
                              <td className="border p-2">Acier zingué, longueur 30 mm, pas standard métrique</td>
                              <td className="border p-2">Fixation robuste et fiable, standard ISO</td>
                            </tr>
                            <tr>
                              <td className="border p-2">STD-003</td>
                              <td className="border p-2">Moteur DC</td>
                              <td className="border p-2 text-center">1</td>
                              <td className="border p-2">12V, 30W, 3000 rpm</td>
                              <td className="border p-2">Déjà disponible, fiable, consommation modérée</td>
                            </tr>
                            <tr>
                              <td className="border p-2">STD-004</td>
                              <td className="border p-2">Capteur de couleur - TCS34725</td>
                              <td className="border p-2 text-center">1</td>
                              <td className="border p-2">Détection RGB, faible consommation, temps de réponse rapide</td>
                              <td className="border p-2">Disponible et adapté à l'application</td>
                            </tr>
                            <tr>
                              <td className="border p-2">STD-005</td>
                              <td className="border p-2">Capteur laser - KY008</td>
                              <td className="border p-2 text-center">2</td>
                              <td className="border p-2">Portée 10 cm, précision ±1 mm</td>
                              <td className="border p-2">Disponible et précis pour la détection d'objets</td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                    </article>
                    
                    <article id="fixations-ajustements">
                      <h3 className="text-lg font-semibold mb-4">4.3 Systèmes de fixation et ajustements mécaniques</h3>
                      <p className="mb-4">L'assemblage du convoyeur repose sur des fixations mécaniques simples et standardisées (vis, écrous, rondelles) associées à des ajustements pensés dès la modélisation. L'objectif est de garantir un montage fiable, démontable et précis, sans nécessiter de pièces spécifiques complexes.</p>
                      
                      <p className="mb-4">Les pièces imprimées intègrent directement des <strong>alésages et perçages</strong> adaptés aux dimensions des vis et aux écrous standards utilisés (M3, M4 selon les zones). Ces emplacements sont dimensionnés avec un léger <strong>jeu fonctionnel</strong> (≈ +0,3 mm sur le diamètre nominal) afin de compenser les tolérances d'impression et d'éviter tout forçage lors du montage.</p>
                      
                      <p className="mb-4">Les <strong>roulements 6202-2RS</strong> sont logés dans des alésages précis (tolérance serrée pour maintien radial), avec un épaulement interne empêchant leur déplacement axial. Le maintien est complété par des <em>couvercles à lèvres</em> fixés par vis, assurant à la fois la protection contre les poussières et la tenue mécanique.</p>
                      
                      <div className="bg-blue-50 dark:bg-blue-900/20 border-l-4 border-blue-600 p-4">
                        <h4 className="font-semibold text-blue-800 dark:text-blue-200 mb-2">Points clés de conception</h4>
                        <ul className="text-sm text-blue-700 dark:text-blue-300 space-y-1">
                          <li>Utilisation de vis et écrous standard M3/M4 pour l'ensemble du montage</li>
                          <li>Jeu fonctionnel prévu dès la modélisation pour faciliter l'assemblage</li>
                          <li>Logements de roulements précis avec épaulement interne</li>
                          <li>Maintien axial des roulements par couvercles vissés</li>
                          <li>Glissières à lumière oblongue pour réglage de tension de bande</li>
                          <li>Renforts aux points de fixation pour éviter la déformation</li>
                        </ul>
                      </div>
                    </article>
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
                  <CardContent className="p-6 space-y-6">
                    <article id="parametres-impression">
                      <h3 className="text-lg font-semibold mb-4">5.1 Paramètres d'impression 3D (matériaux, remplissage, orientation)</h3>
                      <p className="mb-4">Avant de détailler les réglages, voici un tableau récapitulatif des pièces à imprimer en 3D. Il reprend le principe d'une nomenclature (BOM) mais appliquée uniquement aux éléments imprimés. Chaque ligne précise la référence, la description, le matériau recommandé, la quantité et l'orientation d'impression.</p>
                      
                      <div className="overflow-x-auto" suppressHydrationWarning={true}>
                        <table className="w-full text-sm border-collapse">
                          <thead>
                            <tr className="bg-teal-600 text-white">
                              <th className="border border-teal-600 p-3">Réf.</th>
                              <th className="border border-teal-600 p-3">Nom de la pièce</th>
                              <th className="border border-teal-600 p-3">Qté</th>
                              <th className="border border-teal-600 p-3">Matériau recommandé</th>
                              <th className="border border-teal-600 p-3">Orientation conseillée</th>
                              <th className="border border-teal-600 p-3">Remarques</th>
                            </tr>
                          </thead>
                          <tbody>
                            <tr>
                              <td className="border p-2">P01</td>
                              <td className="border p-2">Partie gauche - jointure</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PLA / PETG</td>
                              <td className="border p-2">Face plane sur plateau</td>
                              <td className="border p-2">Alignement précis avec partie droite</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P02</td>
                              <td className="border p-2">Partie droite - jointure</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PLA / PETG</td>
                              <td className="border p-2">Face plane sur plateau</td>
                              <td className="border p-2">Prévoir jeu de 0,2 mm pour emboîtement</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P03</td>
                              <td className="border p-2">Couvercle à lèvres gauche</td>
                              <td className="border p-2">2</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Face extérieure vers le haut</td>
                              <td className="border p-2">Surface visible, soigner finition</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P04</td>
                              <td className="border p-2">Couvercle à lèvres droit</td>
                              <td className="border p-2">2</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Face extérieure vers le haut</td>
                              <td className="border p-2">Surface visible, soigner finition</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P05</td>
                              <td className="border p-2">Partie 2 - bâtis 1</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PETG</td>
                              <td className="border p-2">Face de base sur plateau</td>
                              <td className="border p-2">Pièce structurelle</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P06</td>
                              <td className="border p-2">Partie 2 - bâtis 2</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PETG</td>
                              <td className="border p-2">Face de base sur plateau</td>
                              <td className="border p-2">Pièce structurelle</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P07</td>
                              <td className="border p-2">Paliers en U - glissière</td>
                              <td className="border p-2">2</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Ouverture vers le haut</td>
                              <td className="border p-2">Prévoir jeu fonctionnel</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P08</td>
                              <td className="border p-2">Support capteur couleur</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Face capteur vers le haut</td>
                              <td className="border p-2">Respect passage de câble</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P09</td>
                              <td className="border p-2">Support capteur laser</td>
                              <td className="border p-2">4</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Face capteur vers le haut</td>
                              <td className="border p-2">Alignement précis pour détection</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P10</td>
                              <td className="border p-2">Tambour moteur</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PLA / PETG</td>
                              <td className="border p-2">Axe horizontal</td>
                              <td className="border p-2">Alésage serré pour roulement</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P11</td>
                              <td className="border p-2">Tambour non moteur</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Axe horizontal</td>
                              <td className="border p-2">Identique P10 sans interface moteur</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P12</td>
                              <td className="border p-2">Partie 1 - châssis 1 - gauche</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PETG</td>
                              <td className="border p-2">Face base sur plateau</td>
                              <td className="border p-2">Partie structurelle</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P13</td>
                              <td className="border p-2">Partie 1 - châssis 1 - droite</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PETG</td>
                              <td className="border p-2">Face base sur plateau</td>
                              <td className="border p-2">Partie structurelle</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P14</td>
                              <td className="border p-2">Partie 1 - châssis 2 - gauche</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PETG</td>
                              <td className="border p-2">Face base sur plateau</td>
                              <td className="border p-2">Partie structurelle</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P15</td>
                              <td className="border p-2">Partie 1 - châssis 2 - droite</td>
                              <td className="border p-2">1</td>
                              <td className="border p-2">PETG</td>
                              <td className="border p-2">Face base sur plateau</td>
                              <td className="border p-2">Partie structurelle</td>
                            </tr>
                            <tr>
                              <td className="border p-2">P16</td>
                              <td className="border p-2">Cube</td>
                              <td className="border p-2">4</td>
                              <td className="border p-2">PLA</td>
                              <td className="border p-2">Face plane sur plateau</td>
                              <td className="border p-2">Éviter surchauffe pour précision</td>
                            </tr>
                          </tbody>
                        </table>
                      </div>
                      
                      <h4 className="text-base font-semibold mt-8 mb-4">Cohérence des paramètres</h4>
                      <p className="mb-4">Les réglages d'impression sont choisis pour équilibrer <strong>précision</strong>, <strong>solidité</strong> et <strong>temps de production</strong>. Les pièces soumises à des contraintes mécaniques (P01, P03) reçoivent plus de périmètres et un remplissage plus dense (30 %), tandis que les éléments moins sollicités peuvent être allégés (15–20 %).</p>
                      
                      <div className="bg-teal-50 dark:bg-teal-900/20 border-l-4 border-teal-600 p-4 mb-6">
                        <ul className="text-sm space-y-1">
                          <li>Hauteur de couche : <strong>0,20 mm</strong> (polyvalence)</li>
                          <li>Largeur de ligne : <strong>0,4 mm</strong> (buse standard)</li>
                          <li>Remplissage : <strong>20–30 %</strong></li>
                          <li>Épaisseur parois : <strong>≥ 2 mm</strong></li>
                          <li>Supports : uniquement pour porte-à-faux &gt; 45°</li>
                          <li>Jeu fonctionnel : <strong>≥ 0,3 mm</strong> sur glissières et emboîtements</li>
                        </ul>
                      </div>
                      
                      <h4 className="text-base font-semibold mb-4">Aperçu après slicing (dans le logiciel Prusa)</h4>
                      
                      <h5 className="font-medium mb-3">Images illustrant des problèmes d'impression</h5>
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-10.png" alt="Slicing raté 1" width={400} height={300} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing raté 1 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-11.png" alt="Slicing raté 2" width={400} height={300} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing raté 2 - PrusaSlicer</figcaption>
                        </figure>
                      </div>
                      <p className="mb-4">Ces deux images montrent clairement les problèmes que nous avons eu lors de notre première impression où notre modèle n'arrivait pas à s'intégrer et se poser correctement sur le plateau. Ce qui nous a donc amené à découper en plusieurs morceaux.</p>
                      
                      <div className="flex justify-center mb-6">
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-9.png" alt="Slicing raté 3" width={400} height={300} className="rounded-md border" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing raté 3 - PrusaSlicer</figcaption>
                        </figure>
                      </div>
                      <p className="mb-8">Ici plus haut, on voit clairement que les supports débordent du plateau. Nous avons également procédé à un découpage pour que ça puisse être imprimé.</p>
                      
                      <h5 className="font-medium mb-3">Illustrations de nos slicings réussis et les infos supplémentaires utiles (quantité de matière / temps d'impression)</h5>
                      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4 mb-6">
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-3.png" alt="Slicing 1" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 1 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-4.png" alt="Slicing 2" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 2 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-5.png" alt="Slicing 3" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 3 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-6.png" alt="Slicing 4" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 4 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-7.png" alt="Slicing 5" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 5 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-8.png" alt="Slicing 6" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 6 - PrusaSlicer</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-1.png" alt="Slicing Bambulab 1" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 1 - Bambulab</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/impression_bambulab-2.png" alt="Slicing Bambulab 2" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Slicing 2 - Bambulab</figcaption>
                        </figure>
                      </div>
                      
                      <p className="mb-4">Ces aperçus permettent de valider visuellement l'orientation des pièces, la répartition du remplissage, et la présence éventuelle de supports. Ce contrôle évite les erreurs avant lancement de l'impression.</p>
                      <p>En suivant cette méthodologie, les pièces imprimées sont directement prêtes à être utilisées ou nécessitent seulement un ébavurage minimal. On peut alors passer au post-traitement et à l'assemblage final.</p>
                    </article>
                    
                    <article id="post-traitement">
                      <h3 className="text-lg font-semibold mb-4">5.2 Post-traitement et assemblage final</h3>
                      <p className="mb-4">Une fois les pièces imprimées, plusieurs étapes de finition et de montage sont nécessaires pour assurer la précision d'ajustement, la fluidité des mouvements et la fiabilité mécanique du convoyeur.</p>
                      
                      <h4 className="text-base font-semibold mb-3">A. Préparation des pièces imprimées</h4>
                      <ol className="list-decimal pl-5 space-y-2 mb-6">
                        <li>Ébavurage léger des arêtes afin d'éliminer toute irrégularité due à l'impression.</li>
                        <li>Ponçage localisé des zones d'emboîtement ou de glissière (grain fin).</li>
                        <li>Nettoyage des logements de roulements et axes pour garantir un ajustement précis.</li>
                        <li>Retrait soigneux des éventuels supports d'impression.</li>
                      </ol>
                      
                      <h4 className="text-base font-semibold mb-3">B. Assemblage mécanique</h4>
                      <ol className="list-decimal pl-5 space-y-2 mb-6">
                        <li>Insertion des roulements dans les tambours (pièces P01 et P02).</li>
                        <li>Fixation du tambour moteur sur le support P03 à l'aide de vis M4.</li>
                        <li>Montage du tambour libre à l'extrémité opposée.</li>
                        <li>Installation des glissières P04 avec réglage du jeu latéral.</li>
                        <li>Vérification de l'alignement global des axes et de la bande transporteuse.</li>
                      </ol>
                      
                      <h4 className="text-base font-semibold mb-3">C. Installation des éléments fonctionnels</h4>
                      <ol className="list-decimal pl-5 space-y-2 mb-6">
                        <li>Montage et raccordement du moteur DC.</li>
                        <li>Fixation des supports capteurs et mise en place des capteurs optiques / laser.</li>
                        <li>Pose et tension de la bande transporteuse.</li>
                        <li>Test manuel de rotation pour vérifier l'absence de frottements ou points durs.</li>
                      </ol>
                      
                      <h4 className="text-base font-semibold mb-3">D. Contrôles finaux</h4>
                      <ul className="list-disc pl-5 space-y-1 mb-6">
                        <li>Tambours alignés et bande centrée.</li>
                        <li>Absence de vibrations ou frottements excessifs.</li>
                        <li>Capteurs correctement positionnés et opérationnels.</li>
                        <li>Rotation fluide et stable lors de la mise sous tension.</li>
                      </ul>
                      
                      <h4 className="text-base font-semibold mb-3">E. Assemblage final — Illustrations</h4>
                      <p className="mb-4">Les photos suivantes présentent le convoyeur complètement assemblé et fonctionnel, après toutes les étapes de post-traitement et d'ajustement.</p>
                      
                      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4 mb-6">
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_1.jpg" alt="Convoyeur réel 1" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 1</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_2.jpg" alt="Convoyeur réel 2" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 2</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_3.jpg" alt="Convoyeur réel 3" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 3</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_4.jpg" alt="Convoyeur réel 4" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 4</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_5.jpg" alt="Convoyeur réel 5" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 5</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_6.jpg" alt="Convoyeur réel 6" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 6</figcaption>
                        </figure>
                        <figure className="text-center">
                          <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/assets/imgs/conv-real-img_7.jpg" alt="Convoyeur réel 7" width={300} height={250} className="rounded-md border mx-auto" unoptimized/>
                          <figcaption className="text-xs text-gray-600 mt-2">Vue du convoyeur physique assemblé - 7</figcaption>
                        </figure>
                      </div>
                      
                      <h4 className="text-base font-semibold mb-3">Conclusion de la fabrication</h4>
                      <p>La phase de fabrication, incluant l'impression 3D, le post-traitement et l'assemblage final, a permis de concrétiser la conception du convoyeur de manière fidèle aux modèles numériques. Les choix de paramètres d'impression ont garanti la solidité des pièces tout en optimisant le temps de production, et les ajustements manuels ont permis d'obtenir un assemblage fluide et fiable.</p>
                      <p>Les premiers tests à vide ont confirmé la bonne rotation des tambours, le centrage correct de la bande transporteuse et l'efficacité des systèmes de fixation, validant ainsi la cohérence entre la conception initiale et le produit final.</p>
                    </article>
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
                  <CardContent className="p-6 space-y-6">
                    <p>Au cours de la réalisation, plusieurs imprévus techniques et contraintes pratiques sont apparus, nécessitant des ajustements rapides. Afin de garder une trace claire de ces étapes de résolution, le tableau ci-dessous récapitule les principaux problèmes rencontrés, leur impact potentiel sur le projet et les solutions qui ont été mises en place. Cette synthèse servira de référence pour anticiper ou éviter ces difficultés lors de futures itérations du convoyeur.</p>
                    
                    <div className="overflow-x-auto" suppressHydrationWarning={true}>
                      <table className="w-full text-sm border-collapse">
                        <thead>
                          <tr className="bg-orange-100 dark:bg-orange-900/30">
                            <th className="border p-3 text-left">Problème rencontré</th>
                            <th className="border p-3 text-left">Impact</th>
                            <th className="border p-3 text-left">Solution mise en place</th>
                          </tr>
                        </thead>
                        <tbody>
                          <tr>
                            <td className="border p-3">Tolérances d'impression imprécises sur les logements de roulements</td>
                            <td className="border p-3">Montage difficile, risque de jeu ou de blocage</td>
                            <td className="border p-3">Ajustement des modèles 3D avec un jeu fonctionnel de 0,3 mm et test d'impression à blanc</td>
                          </tr>
                          <tr>
                            <td className="border p-3">Alignement imparfait entre tambours et châssis</td>
                            <td className="border p-3">Mouvement de la bande non rectiligne</td>
                            <td className="border p-3">Ajout de glissières de guidage et contrôle au montage avec un gabarit imprimé</td>
                          </tr>
                          <tr>
                            <td className="border p-3">Fixations des capteurs peu robustes</td>
                            <td className="border p-3">Risque de décalage de lecture des capteurs</td>
                            <td className="border p-3">Renforcement des supports et utilisation de vis M3 avec rondelles</td>
                          </tr>
                          <tr>
                            <td className="border p-3">Câblage trop tendu autour du moteur</td>
                            <td className="border p-3">Usure prématurée des fils, risque de rupture</td>
                            <td className="border p-3">Allongement des câbles et fixation par colliers nylon</td>
                          </tr>
                          <tr>
                            <td className="border p-3">Bruit excessif du système en fonctionnement</td>
                            <td className="border p-3">Inconfort d'utilisation, vibrations</td>
                            <td className="border p-3">Ajout de silentblocs sous le châssis et lubrification des pièces en contact</td>
                          </tr>
                        </tbody>
                      </table>
                    </div>
                    
                    <article id="contraintes-materiaux">
                      <h3 className="text-lg font-semibold mb-4">6.1 Contraintes liées aux matériaux</h3>
                      <p className="mb-4">Le choix du matériau principal pour les pièces imprimées, le PLA, a été dicté par sa disponibilité, sa facilité d'impression et son coût réduit. Cependant, le PLA présente une faible résistance à la chaleur et une rigidité pouvant provoquer des fissures en cas de contraintes mécaniques importantes. Pour pallier ces limites :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li>Renforcement des zones sollicitées par augmentation de l'épaisseur des parois.</li>
                        <li>Ajout de nervures internes pour limiter la flexion.</li>
                        <li>Orientation stratégique des impressions pour optimiser la résistance dans l'axe des efforts.</li>
                      </ul>
                    </article>
                    
                    <article id="ajustements-tolerances">
                      <h3 className="text-lg font-semibold mb-4">6.2 Ajustements mécaniques et tolérances</h3>
                      <p className="mb-4">Les impressions 3D présentent souvent des variations dimensionnelles minimes mais critiques lors de l'assemblage. Les logements de roulements, les guides et les logements d'axe ont nécessité des ajustements répétés. Les solutions appliquées :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li>Application d'un jeu fonctionnel de 0,2 à 0,3 mm pour les pièces mobiles.</li>
                        <li>Ébavurage systématique après impression pour éliminer les aspérités.</li>
                        <li>Utilisation de cales fines pour compenser les écarts lors de l'assemblage.</li>
                      </ul>
                    </article>
                    
                    <article id="contraintes-electroniques">
                      <h3 className="text-lg font-semibold mb-4">6.3 Adaptation aux contraintes électroniques</h3>
                      <p className="mb-4">L'intégration des capteurs et du moteur a imposé des contraintes supplémentaires :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li>Positionnement précis des capteurs pour garantir une lecture fiable.</li>
                        <li>Prévoir des chemins de câbles sécurisés afin d'éviter les frottements avec les pièces en mouvement.</li>
                        <li>Stabilisation du moteur sur silentblocs pour réduire les vibrations et protéger les composants électroniques.</li>
                      </ul>
                    </article>
                    
                    <article id="conclusion-section6">
                      <h3 className="text-lg font-semibold mb-4">Bref conclusion</h3>
                      <p>La réalisation de ce convoyeur a nécessité une approche flexible, combinant modélisation, prototypage et ajustements successifs. Les difficultés rencontrées ont permis de renforcer la robustesse du design et d'optimiser les choix techniques, garantissant un fonctionnement fluide et fiable. Cette phase d'adaptation constitue un retour d'expérience précieux pour toute évolution future du système.</p>
                    </article>
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
                  <CardContent className="p-6 space-y-6">
                    <article id="evaluation-systeme">
                      <h3 className="text-lg font-semibold mb-4">7.1 Évaluation du système</h3>
                      <p className="mb-4">Le convoyeur développé répond globalement aux exigences initiales formulées lors de la phase de conception. Les tests effectués ont confirmé :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li>La bonne stabilité mécanique et le maintien d'un mouvement fluide de la bande.</li>
                        <li>Un alignement précis des composants, minimisant les pertes d'efficacité.</li>
                        <li>Une intégration cohérente des capteurs et du moteur, garantissant un fonctionnement fiable.</li>
                        <li>Une fabrication optimisée pour des moyens d'impression 3D de bureau, sans recours à des procédés industriels lourds.</li>
                      </ul>
                      <p>Malgré quelques ajustements nécessaires en cours de route, le système final présente un niveau de performance satisfaisant et démontre la faisabilité d'un convoyeur compact et modulaire en fabrication additive.</p>
                    </article>
                    
                    <article id="pistes-amelioration">
                      <h3 className="text-lg font-semibold mb-4">7.2 Pistes d'amélioration</h3>
                      <p className="mb-4">Les résultats obtenus ouvrent la voie à plusieurs optimisations futures :</p>
                      <ul className="list-disc pl-5 space-y-1 mb-4">
                        <li><strong>Matériaux :</strong> tester l'utilisation de PETG ou d'ABS pour améliorer la résistance thermique et mécanique.</li>
                        <li><strong>Réduction du bruit :</strong> intégrer davantage de pièces amortissantes pour un fonctionnement plus silencieux.</li>
                        <li><strong>Modularité :</strong> prévoir des sections démontables pour faciliter la maintenance et le transport.</li>
                        <li><strong>Automatisation :</strong> intégrer un système de contrôle plus avancé avec retour en temps réel sur l'état du convoyeur.</li>
                        <li><strong>Optimisation énergétique :</strong> utiliser un moteur plus efficace ou un contrôle intelligent de vitesse selon la charge.</li>
                      </ul>
                    </article>
                    
                    <article id="cloture">
                      <h3 className="text-lg font-semibold mb-4">Clôture</h3>
                      <p>Ce projet a démontré qu'un convoyeur fonctionnel et robuste pouvait être conçu, fabriqué et assemblé avec des moyens accessibles, tout en respectant des contraintes techniques réelles. L'approche par itérations, combinée à l'analyse des problèmes rencontrés, a permis de développer un système adapté à son usage et évolutif pour des besoins futurs. Ces résultats constituent une base solide pour tout développement ultérieur, que ce soit pour augmenter la capacité, améliorer la précision ou intégrer des fonctionnalités avancées.</p>
                    </article>
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