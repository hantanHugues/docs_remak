"use client"

import { useState, useEffect } from "react"
import Link from "next/link"
import { Button } from "@/components/ui/button"
import { Menu, X, FileText, Trophy, Home } from "lucide-react"
import { cn } from "@/lib/utils"
import { ThemeToggle } from "@/components/theme-toggle"
import { useMounted } from "@/hooks/use-mounted"

export function Navbar() {
  const mounted = useMounted()
  const [isScrolled, setIsScrolled] = useState(false)
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false)

  // ✅ TOUS les Hooks DOIVENT être appelés AVANT tout return conditionnel
  useEffect(() => {
    if (!mounted) return

    const handleScroll = () => {
      setIsScrolled(window.scrollY > 10)
    }
    
    handleScroll()
    window.addEventListener("scroll", handleScroll)
    return () => window.removeEventListener("scroll", handleScroll)
  }, [mounted])

  // ✅ Le return conditionnel vient APRÈS tous les Hooks
  if (!mounted) {
    return (
      <nav className="fixed top-0 left-0 right-0 z-50 bg-background/70 backdrop-blur-3xl border-b border-border/40">
        <div className="container mx-auto px-4">
          <div className="flex items-center justify-between h-16 md:h-20">
            <div className="flex items-center gap-3">
              <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-primary to-chart-1 flex items-center justify-center">
                <Trophy className="w-6 h-6 text-white" />
              </div>
              <div className="hidden md:block">
                <div className="text-lg font-bold bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent">
                  TRC Competition
                </div>
                <div className="text-xs text-muted-foreground">
                  Documentation Technique
                </div>
              </div>
            </div>
            <div className="hidden md:flex items-center gap-2">
              <Button variant="ghost" className="gap-2">
                <Home className="w-4 h-4" />
                Accueil
              </Button>
              <Button variant="ghost" className="gap-2">
                <FileText className="w-4 h-4" />
                Pré-sélection
              </Button>
              <Button variant="ghost" className="gap-2">
                <Trophy className="w-4 h-4" />
                Finale
              </Button>
            </div>
            <div className="md:hidden flex items-center gap-2">
              <Button variant="ghost" size="icon">
                <Menu className="w-5 h-5" />
              </Button>
            </div>
          </div>
        </div>
      </nav>
    )
  }

  return (
    <nav
      className={cn(
        "fixed top-0 left-0 right-0 z-50 transition-all duration-500 ease-out",
        isScrolled
          ? "bg-background/70 backdrop-blur-3xl border-b border-border/40 shadow-2xl shadow-primary/10"
          : "bg-gradient-to-b from-primary/10 via-primary/5 to-transparent backdrop-blur-sm border-b border-transparent",
      )}
    >
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-between h-16 md:h-20">
          <Link href="/" className="flex items-center gap-3 group">
            <div
              className={cn(
                "w-10 h-10 rounded-xl bg-gradient-to-br from-primary to-chart-1 flex items-center justify-center transition-all duration-500",
                isScrolled ? "shadow-xl shadow-primary/40" : "shadow-lg shadow-primary/20",
                "group-hover:scale-110 group-hover:shadow-2xl group-hover:shadow-primary/50",
              )}
            >
              <Trophy className="w-6 h-6 text-white" />
            </div>
            <div className="hidden md:block">
              <div
                className={cn(
                  "text-lg font-bold bg-gradient-to-r from-primary to-chart-1 bg-clip-text text-transparent transition-all duration-500",
                  isScrolled ? "opacity-100" : "opacity-90",
                )}
              >
                TRC Competition
              </div>
              <div
                className={cn(
                  "text-xs text-muted-foreground transition-all duration-500",
                  isScrolled ? "opacity-100" : "opacity-70",
                )}
              >
                Documentation Technique
              </div>
            </div>
          </Link>

          <div className="hidden md:flex items-center gap-2">
            <Link href="/">
              <Button
                variant="ghost"
                className={cn("gap-2 transition-all duration-300", isScrolled && "hover:bg-primary/10")}
              >
                <Home className="w-4 h-4" />
                Accueil
              </Button>
            </Link>
            <Link href="/pre-selection">
              <Button
                variant="ghost"
                className={cn("gap-2 transition-all duration-300", isScrolled && "hover:bg-primary/10")}
              >
                <FileText className="w-4 h-4" />
                Pré-sélection
              </Button>
            </Link>
            <Link href="/finale">
              <Button
                variant="ghost"
                className={cn("gap-2 transition-all duration-300", isScrolled && "hover:bg-primary/10")}
              >
                <Trophy className="w-4 h-4" />
                Finale
              </Button>
            </Link>
            <ThemeToggle />
          </div>

          <div className="md:hidden flex items-center gap-2">
            <ThemeToggle />
            <Button
              variant="ghost"
              size="icon"
              className="md:hidden"
              onClick={() => setIsMobileMenuOpen(!isMobileMenuOpen)}
            >
              {isMobileMenuOpen ? <X className="w-5 h-5" /> : <Menu className="w-5 h-5" />}
            </Button>
          </div>
        </div>

        {isMobileMenuOpen && (
          <div className="md:hidden py-4 border-t border-border/40 space-y-2 animate-in slide-in-from-top">
            <Link href="/" onClick={() => setIsMobileMenuOpen(false)}>
              <Button variant="ghost" className="w-full justify-start gap-2">
                <Home className="w-4 h-4" />
                Accueil
              </Button>
            </Link>
            <Link href="/pre-selection" onClick={() => setIsMobileMenuOpen(false)}>
              <Button variant="ghost" className="w-full justify-start gap-2">
                <FileText className="w-4 h-4" />
                Pré-sélection
              </Button>
            </Link>
            <Link href="/finale" onClick={() => setIsMobileMenuOpen(false)}>
              <Button variant="ghost" className="w-full justify-start gap-2">
                <Trophy className="w-4 h-4" />
                Finale
              </Button>
            </Link>
          </div>
        )}
      </div>
    </nav>
  )
}