"use client"

import { useEffect } from 'react'

export function ScrollIndicator() {
  useEffect(() => {
    let scrollTimeout: NodeJS.Timeout

    const handleScroll = () => {
      // Add scrolling class to body
      document.body.classList.add('scrolling')
      
      // Clear existing timeout
      clearTimeout(scrollTimeout)
      
      // Remove scrolling class after scroll ends
      scrollTimeout = setTimeout(() => {
        document.body.classList.remove('scrolling')
      }, 1000) // Hide scrollbar 1 second after scroll stops
    }

    // Add scroll event listener
    window.addEventListener('scroll', handleScroll, { passive: true })
    
    // Also handle scroll on any scrollable element
    const handleElementScroll = (e: Event) => {
      const target = e.target as HTMLElement
      target.classList.add('scrolling')
      
      clearTimeout(scrollTimeout)
      scrollTimeout = setTimeout(() => {
        target.classList.remove('scrolling')
      }, 1000)
    }

    // Add scroll listeners to all potentially scrollable elements
    const scrollableElements = document.querySelectorAll('*')
    scrollableElements.forEach(element => {
      element.addEventListener('scroll', handleElementScroll, { passive: true })
    })

    // Cleanup
    return () => {
      window.removeEventListener('scroll', handleScroll)
      scrollableElements.forEach(element => {
        element.removeEventListener('scroll', handleElementScroll)
      })
      clearTimeout(scrollTimeout)
    }
  }, [])

  return null // This component doesn't render anything
}
