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
  ArrowLeft,
  ArrowRight,
  Code,
  BookOpen,
  GitBranch,
  PlayCircle,
  FolderArchive,
  Target,
  CheckCircle,
  BarChart2,
  AlertTriangle,
  ExternalLink,
  FileText,
  Settings,
  Layers,
  Bot,
  Cpu,
  Wifi,
  History
} from "lucide-react";
import Link from "next/link";
import Image from "next/image";

export default function IndustrialDashboardDocPage() {
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
                      Projet
                    </Link>
                    <span>/</span>
                    <span>Dashboard Industriel</span>
                  </div>

                  <div className="flex items-center gap-4 mb-6">
                    <div className="w-12 h-12 rounded-lg bg-gray-100 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 flex items-center justify-center">
                      <BarChart2 className="w-6 h-6 text-gray-600 dark:text-gray-400" />
                    </div>
                    <div>
                      <h1 className="text-3xl md:text-4xl font-bold">Documentation du Projet : Dashboard Industriel de Tri des Déchets</h1>
                      <p className="text-muted-foreground">Processus de développement, de débogage et d&apos;amélioration du projet.</p>
                    </div>
                  </div>

                  <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">Full-Stack</Badge>
                    <Badge variant="outline">React</Badge>
                    <Badge variant="outline">Node.js</Badge>
                    <Badge variant="outline">MongoDB</Badge>
                    <Badge variant="outline">Socket.IO</Badge>
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
                      <Target className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      1. Objectif du Projet
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      L&apos;objectif principal est de développer un système de supervision en temps réel pour une chaîne de tri de déchets. Le système doit permettre de visualiser les données envoyées par un convoyeur (simulé ou via un microcontrôleur ESP32), de les stocker, et d&apos;interagir avec un assistant IA pour obtenir des analyses et des rapports.
                    </p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={50}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <ExternalLink className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      2. Accès à l&apos;Application
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">L&apos;application est déployée et accessible en ligne à l&apos;adresse suivante :</p>
                    <Link href="https://convoyeur-front-r5y5.vercel.app/" target="_blank" rel="noopener noreferrer" className="text-blue-600 dark:text-blue-400 hover:underline break-all">
                      https://convoyeur-front-r5y5.vercel.app/
                    </Link>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      3. Instructions d&apos;Installation et de Lancement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <h3 className="font-semibold">Prérequis</h3>
                    <ul className="list-disc pl-5 text-sm"><li>Node.js, npm, MongoDB Atlas, Clé API Google Gemini.</li></ul>
                    <h3 className="font-semibold">Étape 1 : Configuration du Backend (`/server`)</h3>
                    <div className="bg-gray-900 rounded p-2 text-sm text-white font-mono">cd server && npm install</div>
                    <p>Créer et remplir un fichier <code>.env</code> avec <code>MONGODB_URI</code> et <code>GEMINI_API_KEY</code>.</p>
                    <h3 className="font-semibold">Étape 2 : Configuration du Frontend (`/client`)</h3>
                    <div className="bg-gray-900 rounded p-2 text-sm text-white font-mono">cd client && npm install</div>
                    <h3 className="font-semibold">Étape 3 : Lancement de l&apos;Application (3 terminaux)</h3>
                    <p>Terminal 1 (Backend): <code>cd server && npm start</code></p>
                    <p>Terminal 2 (Frontend): <code>cd client && npm start</code></p>
                    <p>Terminal 3 (Simulateur): <code>cd server && node simulate.js</code></p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      4. Guide de Déploiement
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <h3 className="font-semibold">Étape 1 : Préparer le Frontend pour la Production</h3>
                    <p>Configurer l&apos;URL du backend via une variable d&apos;environnement <code>REACT_APP_BACKEND_URL</code> et compiler avec <code>npm run build</code>.</p>
                    <h3 className="font-semibold">Étape 2 : Déployer le Frontend sur Vercel (Déjà effectué)</h3>
                    <p>Le frontend a été déployé avec succès sur Vercel : <Link href="https://convoyeur-front-r5y5.vercel.app/" className="text-blue-600 dark:text-blue-400">https://convoyeur-front-r5y5.vercel.app/</Link></p>
                    <h3 className="font-semibold">Étape 3 : Déployer le Backend</h3>
                    <p>Héberger sur Render ou Heroku. Configurer le service Node.js avec les commandes de build (`npm install`) et de démarrage (`node index.js`), et ajouter les variables d&apos;environnement (`MONGODB_URI`, `GEMINI_API_KEY`, etc.).</p>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <History className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      5. Chronologie du Développement et Résolution des Problèmes
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <h3 className="font-semibold">Phase 1 : Le simulateur ne communique pas</h3>
                    <p className="text-sm"><strong>Problème :</strong> Le serveur renvoyait `404 Not Found`. <strong>Solution :</strong> Ajout d&apos;une route `POST /api/event` qui crée un enregistrement en base de données et émet un événement `newEvent` via Socket.IO.</p>
                    <h3 className="font-semibold">Phase 2 : Améliorer l&apos;efficacité du développement</h3>
                    <p className="text-sm"><strong>Problème :</strong> Redémarrage manuel du serveur. <strong>Solution :</strong> Intégration de `nodemon` avec un script `npm start`.</p>
                    <h3 className="font-semibold">Phase 3 : Sécuriser et centraliser la configuration</h3>
                    <p className="text-sm"><strong>Problème :</strong> Clés en dur dans le code. <strong>Solution :</strong> Déplacement de toutes les clés sensibles dans un fichier `.env`.</p>
                    <h3 className="font-semibold">Phase 4 & 6 : Diagnostic des erreurs de services externes</h3>
                    <p className="text-sm"><strong>Problème MongoDB :</strong> Erreurs `ETIMEDOUT`. <strong>Solution :</strong> Mise à jour de la liste d&apos;accès IP sur MongoDB Atlas.</p>
                    <p className="text-sm"><strong>Problème Gemini :</strong> Erreurs `503` et `404`. <strong>Solution :</strong> Utilisation du nom de modèle correct (`gemini-1.5-flash-latest`) et gestion des surcharges de service.</p>
                    <h3 className="font-semibold">Phase 5 : Rendre l&apos;application entièrement responsive</h3>
                    <p className="text-sm"><strong>Défi 1 (Navigation cassée) :</strong> Résolu en remplaçant la mise en page Flexbox par CSS Grid (`grid-template-rows: auto 1fr;`) dans le composant des onglets.</p>
                    <p className="text-sm"><strong>Défi 2 (Widgets vides sur mobile) :</strong> Résolu en ajoutant une hauteur minimale (`min-height: 250px`) aux conteneurs des widgets via une media query.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      6. Évolution des Objectifs et Étapes Suivantes
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <h3 className="font-semibold">Objectifs Initiaux (Atteints)</h3>
                    <ul className="list-disc pl-5 text-sm">
                      <li>Stabiliser la connexion à la base de données.</li>
                      <li>Améliorer la robustesse du code.</li>
                      <li>Finaliser l&apos;intégration ESP32 avec un code d&apos;exemple.</li>
                    </ul>
                    <h3 className="font-semibold">Nouveaux Objectifs et Vision à Long Terme</h3>
                    <ol className="list-decimal pl-5 text-sm">
                      <li><strong>Déploiement en Production :</strong> Rendre l&apos;application accessible en continu.</li>
                      <li><strong>Enrichissement de l&apos;IA :</strong> Détection proactive d&apos;anomalies et notifications.</li>
                      <li><strong>Tests et Optimisation :</strong> Mettre en place des tests automatisés et analyser les performances.</li>
                      <li><strong>Amélioration Continue de l&apos;UX :</strong> Recueillir les retours utilisateurs.</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      7. Architecture Détaillée et Flux de Données
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-4">
                    <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/media/Untitled diagram _ Mermaid Chart-2025-07-12-143439.png" alt="Schéma d'architecture technique" width={800} height={450} className="rounded-md border mx-auto"/>
                    <h3 className="font-semibold">Flux d&apos;un événement :</h3>
                    <ol className="list-decimal pl-5 space-y-2 text-sm">
                      <li><strong>Génération (ESP32/Simulateur) :</strong> Un événement (`{JSON.stringify({color:"blue"})}`) est envoyé via `POST` à `/api/event`.</li>
                      <li><strong>Réception et Traitement (Serveur Node.js) :</strong> Express intercepte la requête, Mongoose sauvegarde l&apos;événement dans MongoDB, puis Socket.IO émet un message `newEvent` à tous les clients.</li>
                      <li><strong>Affichage (Client React) :</strong> L&apos;écouteur Socket.IO `socket.on(&apos;newEvent&apos;, ...)` reçoit le message, met à jour l&apos;état de l&apos;application, et React met à jour l&apos;interface en temps réel.</li>
                    </ol>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      8. Justification des Choix Technologiques
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-2 text-sm">
                    <p><strong>Node.js :</strong> Idéal pour les applications I/O-intensives (IoT).</p>
                    <p><strong>Express.js :</strong> Standard de facto, minimaliste et robuste.</p>
                    <p><strong>MongoDB :</strong> Base de données NoSQL flexible et orientée document.</p>
                    <p><strong>Mongoose :</strong> Ajoute une couche de structuration et de validation.</p>
                    <p><strong>React.js :</strong> Bibliothèque de référence pour des UI modernes et réactives.</p>
                    <p><strong>Socket.IO :</strong> Solution complète pour la communication bidirectionnelle en temps réel.</p>
                    <p><strong>JSON Web Token (JWT) :</strong> Méthode standard, &quot;stateless&quot; et sécurisée pour l&apos;authentification.</p>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      9. Fonctionnalités Clés : De la Vision à l&apos;Implémentation
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6 space-y-6">
                    <div>
                      <h3 className="font-semibold">a. Dashboard Principal et Widgets Temps Réel</h3>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/media/DEASHBOARD.png" alt="Dashboard principal" width={700} height={400} className="rounded-md border my-2"/>
                    </div>
                    <div>
                      <h3 className="font-semibold">b. Visualisations et Statistiques</h3>
                      <p className="text-sm">Graphiques agrégés (camemberts, histogrammes) avec Recharts.</p>
                    </div>
                    <div>
                      <h3 className="font-semibold">c. Page d&apos;Historique, Exports et Rapports</h3>
                      <p className="text-sm">Interface épurée avec en-tête de tableau fixe, chargement intelligent et exports PDF, CSV, JSON générés côté serveur.</p>
                    </div>
                    <div>
                      <h3 className="font-semibold">d. Assistant IA Conversationnel (Gemini)</h3>
                      <p className="text-sm">Le serveur agit comme un intermédiaire intelligent, construisant un prompt riche avec des données en temps réel avant d&apos;interroger Gemini. L&apos;interface intègre la Web Speech API.</p>
                      <Image src="/2025-Team-IFRI-Docs/Documentation/test-final/media/ASSISTANTia.png" alt="Assistant IA" width={700} height={400} className="rounded-md border my-2"/>
                    </div>
                    <div>
                      <h3 className="font-semibold">e. Page de Paramètres Avancés et Sécurisés</h3>
                      <p className="text-sm">Section &quot;Admin&quot; protégée par JWT pour les actions sensibles. Thème clair/sombre fonctionnel.</p>
                      <div className="flex gap-4 my-2"><Image src="/2025-Team-IFRI-Docs/Documentation/test-final/media/PARAMERROUILLÉ.png" alt="Paramètres Admin - Verrouillé" width={300} height={200} className="rounded-md border"/><Image src="/2025-Team-IFRI-Docs/Documentation/test-final/media/ADMINSPACE.png" alt="Paramètres Admin - Déverrouillé" width={300} height={200} className="rounded-md border"/></div>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>

              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <Cpu className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      10. Code Source du Microcontrôleur (ESP32)
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="mb-4">Un code d&apos;exemple pour un ESP32 est fourni. Il utilise une communication HTTP POST simple et robuste pour envoyer les données au format JSON vers le backend.</p>
                    <div className="bg-gray-900 rounded p-4 text-sm text-white font-mono overflow-x-auto">
                      <pre><code>{`#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "VOTRE_SSID_WIFI";
const char* password = "VOTRE_MOT_DE_PASSE_WIFI";
const char* serverName = "http://192.168.1.XX:3001/api/event";

void setup() { /* ... Connexion WiFi ... */ }

void loop() {
  String color = "green"; // Simule la détection
  if(WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
    String jsonPayload = "{\\"color\\":\\"" + color + "\\"}";
    int httpResponseCode = http.POST(jsonPayload);
    // ... Gestion de la réponse ...
    http.end();
  }
  delay(5000);
}`}</code></pre>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <PlayCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      11. Vidéo de Démonstration
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="mb-4">Une vidéo complète présentant le fonctionnement de l&apos;application est disponible ci-dessous.</p>
                    <div className="aspect-video rounded-lg overflow-hidden border">
                      <iframe src="https://www.youtube.com/embed/AmRN43rIMaQ" title="Vidéo de démonstration du projet" frameBorder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowFullScreen className="w-full h-full"></iframe>
                    </div>
                  </CardContent>
                </Card>
              </AnimatedSection>
              
              <AnimatedSection animation="fade-up" delay={100}>
                <Card className="border border-gray-200 dark:border-gray-700">
                  <CardHeader className="bg-white dark:bg-gray-900/50 border-b border-gray-200 dark:border-gray-700">
                    <CardTitle className="flex items-center gap-2 text-gray-900 dark:text-gray-100">
                      <CheckCircle className="w-5 h-5 text-gray-600 dark:text-gray-400" />
                      12. Conclusion et Remerciements
                    </CardTitle>
                  </CardHeader>
                  <CardContent className="p-6">
                    <p className="text-gray-700 dark:text-gray-300">
                      Ce projet a été une excellente démonstration de développement full-stack moderne, intégrant des technologies temps réel, des bases de données cloud, et de l&apos;intelligence artificielle. Chaque défi, du débogage CSS à la stabilisation des connexions externes, a été une opportunité d&apos;apprentissage documentée dans ce fichier.
                    </p>
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
  );
}