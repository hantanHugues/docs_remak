"use client"

import type React from "react"

import { useScrollAnimation } from "@/hooks/use-scroll-animation"
import { cn } from "@/lib/utils"

interface AnimatedSectionProps {
  children: React.ReactNode
  className?: string
  delay?: number
  animation?: "fade-up" | "fade-in" | "slide-left" | "slide-right" | "scale"
}

export function AnimatedSection({ children, className, delay = 0, animation = "fade-up" }: AnimatedSectionProps) {
  const { ref, isVisible } = useScrollAnimation({ threshold: 0.1, triggerOnce: true })

  const animationClasses = {
    "fade-up": "translate-y-12 opacity-0",
    "fade-in": "opacity-0",
    "slide-left": "translate-x-12 opacity-0",
    "slide-right": "-translate-x-12 opacity-0",
    scale: "scale-95 opacity-0",
  }

  const visibleClasses = "translate-y-0 translate-x-0 opacity-100 scale-100"

  return (
    <section
      ref={ref}
      className={cn(
        "transition-all duration-700 ease-out",
        isVisible ? visibleClasses : animationClasses[animation],
        className,
      )}
      style={{ transitionDelay: `${delay}ms` }}
    >
      {children}
    </section>
  )
}
