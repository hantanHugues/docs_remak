"use client"

import type React from "react"
import { useState, useEffect } from "react"
import Link from "next/link"
import { usePathname } from "next/navigation"
import { motion, AnimatePresence } from "framer-motion"
import { ChevronRight, Home, CheckCircle2, Target, Zap } from "lucide-react"
import { cn } from "@/lib/utils"
import { Button } from "@/components/ui/button"

interface SidebarItem {
  title: string
  href: string
  iconName?: string
  items?: SidebarItem[]
}

// Map des icônes disponibles
const iconMap = {
  CheckCircle2,
  Target,
  Zap,
}

interface DocsSidebarProps {
  items: SidebarItem[]
}

export function DocsSidebar({ items }: DocsSidebarProps) {
  const [isExpanded, setIsExpanded] = useState(false)
  const [isMounted, setIsMounted] = useState(false)
  const [expandedItems, setExpandedItems] = useState<Set<string>>(new Set())
  const pathname = usePathname()

  useEffect(() => {
    setIsMounted(true)
    
    // Récupérer l'état sauvegardé depuis localStorage (seulement côté client)
    let autoExpandItems = new Set<string>()
    
    if (typeof window !== 'undefined') {
      const savedExpandedItems = localStorage.getItem('sidebar-expanded-items')
      if (savedExpandedItems) {
        try {
          const parsed = JSON.parse(savedExpandedItems)
          autoExpandItems = new Set(parsed)
        } catch (error) {
          console.warn('Erreur lors de la lecture du localStorage:', error)
        }
      }
    }
    
    // Auto-déplier la section correspondant à la page actuelle (seulement si pas d'état sauvegardé)
    if (autoExpandItems.size === 0) {
      if (pathname.includes('/pre-selection') || pathname.includes('/docs/semaine-')) {
        autoExpandItems.add('Pré-sélection')
      }
      
      // Semaine 1 spécifique
      if (pathname.includes('/docs/semaine-1/')) {
        autoExpandItems.add('Pré-sélection')
        autoExpandItems.add('Pré-sélection-Semaine 1 - Débutants')
        
        if (pathname.includes('/electronique')) {
          autoExpandItems.add('Pré-sélection-Semaine 1 - Débutants-Électronique')
        }
        if (pathname.includes('/it')) {
          autoExpandItems.add('Pré-sélection-Semaine 1 - Débutants-IT')
        }
        if (pathname.includes('/mecanique')) {
          autoExpandItems.add('Pré-sélection-Semaine 1 - Débutants-Mécanique')
        }
      }
      
      // Semaine 2 spécifique
      if (pathname.includes('/docs/semaine-2/')) {
        autoExpandItems.add('Pré-sélection')
        autoExpandItems.add('Pré-sélection-Semaine 2 - Intermédiaires')
        
        if (pathname.includes('/electronique')) {
          autoExpandItems.add('Pré-sélection-Semaine 2 - Intermédiaires-Électronique')
        }
        if (pathname.includes('/it')) {
          autoExpandItems.add('Pré-sélection-Semaine 2 - Intermédiaires-IT')
        }
        if (pathname.includes('/mecanique')) {
          autoExpandItems.add('Pré-sélection-Semaine 2 - Intermédiaires-Mécanique')
        }
      }
      
      // Semaine 3 spécifique
      if (pathname.includes('/docs/semaine-3/')) {
        autoExpandItems.add('Pré-sélection')
        autoExpandItems.add('Pré-sélection-Semaine 3 - Avancés')
        
        if (pathname.includes('/electronique')) {
          autoExpandItems.add('Pré-sélection-Semaine 3 - Avancés-Électronique')
        }
        if (pathname.includes('/it')) {
          autoExpandItems.add('Pré-sélection-Semaine 3 - Avancés-IT')
        }
        if (pathname.includes('/mecanique')) {
          autoExpandItems.add('Pré-sélection-Semaine 3 - Avancés-Mécanique')
        }
      }
      
      if (pathname.includes('/finale')) {
        autoExpandItems.add('Finale')
      }
    }
    
    setExpandedItems(autoExpandItems)
  }, [pathname])

  const toggleExpanded = (itemTitle: string) => {
    const newExpanded = new Set(expandedItems)
    if (newExpanded.has(itemTitle)) {
      newExpanded.delete(itemTitle)
    } else {
      newExpanded.add(itemTitle)
    }
    setExpandedItems(newExpanded)
    
    // Sauvegarder l'état dans localStorage (seulement côté client)
    if (typeof window !== 'undefined') {
      localStorage.setItem('sidebar-expanded-items', JSON.stringify(Array.from(newExpanded)))
    }
  }

  const renderNavItem = (item: SidebarItem, level: number = 0, parentPath: string = '') => {
    const Icon = item.iconName ? iconMap[item.iconName as keyof typeof iconMap] : null
    const isActive = pathname === item.href
    const hasSubItems = item.items && item.items.length > 0
    const itemId = parentPath ? `${parentPath}-${item.title}` : item.title
    const isItemExpanded = expandedItems.has(itemId)

    return (
      <div key={item.title} className="space-y-1">
        <Link href={item.href} prefetch={true}>
          <Button
            variant={isActive ? "secondary" : "ghost"}
            size="sm"
            className={cn(
              "w-full justify-start gap-3 transition-all duration-500 ease-in-out text-sidebar-foreground hover:text-sidebar-primary hover:bg-sidebar-accent",
              !isExpanded && "justify-center px-0",
              isActive &&
                "bg-sidebar-accent text-sidebar-primary hover:bg-sidebar-accent/80 shadow-lg shadow-sidebar-primary/20 font-semibold",
            )}
            onClick={(e) => {
              if (hasSubItems && isExpanded) {
                // Ne pas empêcher la navigation, mais déplier/replier la section
                toggleExpanded(itemId)
                // Laisser le Link gérer la navigation vers la page de présentation
              }
            }}
          >
            {Icon && <Icon className="h-5 w-5 flex-shrink-0" />}
            {!Icon && hasSubItems && (
              <ChevronRight className={cn(
                "h-4 w-4 flex-shrink-0 transition-transform duration-200",
                isItemExpanded && "rotate-90"
              )} />
            )}
            <AnimatePresence>
              {isExpanded && (
                <motion.span
                  className="truncate"
                  initial={{ opacity: 0, x: -10 }}
                  animate={{ opacity: 1, x: 0 }}
                  exit={{ opacity: 0, x: -10 }}
                  transition={{ duration: 0.3, delay: 0.15 }}
                >
                  {item.title}
                </motion.span>
              )}
            </AnimatePresence>
          </Button>
        </Link>

        {/* Sub-items */}
        {hasSubItems && isExpanded && isItemExpanded && (
          <div className="ml-4 pl-4 border-l-2 border-sidebar-primary/40 space-y-1">
            {item.items?.map((subItem) => renderNavItem(subItem, level + 1, itemId))}
          </div>
        )}
      </div>
    )
  }

  // Rendu sans animation pendant l'hydratation
  if (!isMounted) {
    return (
      <aside className="sticky top-16 md:top-20 w-16 h-[calc(100vh-4rem)] md:h-[calc(100vh-5rem)] bg-sidebar/95 backdrop-blur-lg border-r border-sidebar-border/60 z-40 shadow-2xl flex-shrink-0">
        <div className="flex flex-col h-full">
          <div className="p-3 border-b border-sidebar-border/60">
            <Button variant="ghost" size="sm" className="w-full justify-center px-0">
              <Home className="h-5 w-5 flex-shrink-0" />
            </Button>
          </div>
          <div className="flex-1 min-h-0 overflow-y-auto overflow-x-hidden">
            <nav className="p-3 space-y-2">
              {items?.map((item, idx) => {
                const Icon = item.iconName ? iconMap[item.iconName as keyof typeof iconMap] : null
                return (
                  <div key={idx}>
                    <Button variant="ghost" size="sm" className="w-full justify-center px-0">
                      {Icon && <Icon className="h-5 w-5 flex-shrink-0" />}
                    </Button>
                  </div>
                )
              })}
            </nav>
          </div>
        </div>
      </aside>
    )
  }

  return (
    <motion.aside
      className="sticky top-16 md:top-20 h-[calc(100vh-4rem)] md:h-[calc(100vh-5rem)] bg-sidebar/95 backdrop-blur-lg border-r z-40 shadow-lg flex-shrink-0 relative group"
      animate={{ 
        width: isExpanded ? 288 : 64 
      }}
      transition={{ 
        type: "spring", 
        stiffness: 300, 
        damping: 30,
        duration: 0.6
      }}
      onMouseEnter={() => setIsExpanded(true)}
      onMouseLeave={() => setIsExpanded(false)}
    >
      {/* Flèche indicatrice d'expansion */}
      <motion.div
        className="absolute right-1 top-1/2 -translate-y-1/2 pointer-events-none"
        animate={{
          opacity: isExpanded ? 0 : [0.4, 0.8, 0.4],
          x: isExpanded ? -10 : [0, 3, 0],
        }}
        transition={{
          opacity: { duration: 2, repeat: Infinity, ease: "easeInOut" },
          x: { duration: 2, repeat: Infinity, ease: "easeInOut" },
        }}
      >
        <ChevronRight className="h-5 w-5 text-sidebar-primary/70" />
      </motion.div>
      <div className="flex flex-col h-full">
        {/* Home Button */}
        <div className="p-3 border-b border-sidebar-border/60">
          <Link href="/">
            <Button
              variant="ghost"
              size="sm"
              className={cn(
                "w-full justify-start gap-3 hover:bg-sidebar-accent text-sidebar-foreground hover:text-sidebar-primary transition-all duration-500 ease-in-out",
                !isExpanded && "justify-center px-0",
              )}
            >
              <Home className="h-5 w-5 flex-shrink-0" />
              <AnimatePresence>
                {isExpanded && (
                  <motion.span
                    className="font-medium"
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    exit={{ opacity: 0, x: -10 }}
                    transition={{ duration: 0.3, delay: 0.1 }}
                  >
                    Accueil
                  </motion.span>
                )}
              </AnimatePresence>
            </Button>
          </Link>
        </div>

        {/* Navigation Items */}
        <div className="flex-1 min-h-0 overflow-y-auto overflow-x-hidden">
          <nav className="p-3 pb-6 space-y-2">
            {items?.map((item) => renderNavItem(item))}
          </nav>
          <style jsx>{`
            .flex-1::-webkit-scrollbar {
              width: 8px;
            }
            .flex-1::-webkit-scrollbar-track {
              background: transparent;
            }
            .flex-1::-webkit-scrollbar-thumb {
              background: rgba(255, 255, 255, 0.2);
              border-radius: 4px;
            }
            .flex-1::-webkit-scrollbar-thumb:hover {
              background: rgba(255, 255, 255, 0.3);
            }
          `}</style>
        </div>

      </div>
    </motion.aside>
  )
}
