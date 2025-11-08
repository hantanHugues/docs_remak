"use client";

import { Navbar } from "@/components/navbar";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Home, ArrowLeft, FileQuestion } from "lucide-react";
import Link from "next/link";

export default function NotFound() {
  return (
    <div className="min-h-screen bg-background">
      <Navbar />
      
      <div className="flex items-center justify-center min-h-[calc(100vh-4rem)] px-4">
        <Card className="w-full max-w-md text-center">
          <CardHeader className="pb-4">
            <div className="mx-auto w-16 h-16 rounded-full bg-muted flex items-center justify-center mb-4">
              <FileQuestion className="w-8 h-8 text-muted-foreground" />
            </div>
            <CardTitle className="text-2xl">Page non trouvée</CardTitle>
            <CardDescription>
              La page que vous recherchez n'existe pas ou est en cours de développement.
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="text-6xl font-bold text-muted-foreground/20">404</div>
            <div className="flex flex-col sm:flex-row gap-2">
              <Link href="/" className="flex-1">
                <Button variant="default" className="w-full">
                  <Home className="w-4 h-4 mr-2" />
                  Accueil
                </Button>
              </Link>
              <Button 
                variant="outline" 
                onClick={() => window.history.back()}
                className="flex-1"
              >
                <ArrowLeft className="w-4 h-4 mr-2" />
                Retour
              </Button>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
