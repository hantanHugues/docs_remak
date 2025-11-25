"use client";

import Link from "next/link";
import { Button } from "@/components/ui/button";
import { ArrowLeft, ArrowRight } from "lucide-react";
import { usePathname } from "next/navigation";
import { getPageNavigation } from "@/lib/navigation-config";

export function PageNavigation() {
  const pathname = usePathname();
  const navigation = getPageNavigation(pathname);

  if (!navigation) {
    return null;
  }

  const { previous, next } = navigation;

  return (
    <div className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 sticky top-16 md:top-20 z-30">
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-between py-3 max-w-4xl mx-auto">
          {previous ? (
            <Link href={previous.href}>
              <Button variant="ghost" size="sm">
                <ArrowLeft className="w-4 h-4 mr-2" />
                {previous.title}
              </Button>
            </Link>
          ) : (
            <div></div>
          )}
          
          {next ? (
            <Link href={next.href}>
              <Button variant="ghost" size="sm">
                {next.title}
                <ArrowRight className="w-4 h-4 ml-2" />
              </Button>
            </Link>
          ) : (
            <div></div>
          )}
        </div>
      </div>
    </div>
  );
}
