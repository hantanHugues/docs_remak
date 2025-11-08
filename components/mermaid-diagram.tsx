"use client";

import { useEffect, useRef, useState } from 'react';
import mermaid from 'mermaid';
import { ZoomIn, ZoomOut, RotateCcw, Maximize2 } from 'lucide-react';

interface MermaidDiagramProps {
  chart: string;
  className?: string;
}

export function MermaidDiagram({ chart, className = "" }: MermaidDiagramProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const diagramRef = useRef<HTMLDivElement>(null);
  const [scale, setScale] = useState(1);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [isFullscreen, setIsFullscreen] = useState(false);

  useEffect(() => {
    if (diagramRef.current) {
      // Configuration de Mermaid
      mermaid.initialize({
        startOnLoad: true,
        theme: 'dark',
        themeVariables: {
          primaryColor: '#3b82f6',
          primaryTextColor: '#ffffff',
          primaryBorderColor: '#1e40af',
          lineColor: '#6b7280',
          sectionBkgColor: '#1f2937',
          altSectionBkgColor: '#374151',
          gridColor: '#4b5563',
          secondaryColor: '#10b981',
          tertiaryColor: '#f59e0b',
        },
        flowchart: {
          useMaxWidth: false,
          htmlLabels: true,
        },
        stateDiagram: {
          useMaxWidth: false,
        },
        classDiagram: {
          useMaxWidth: false,
        },
      });

      // Générer un ID unique pour le diagramme
      const id = `mermaid-${Math.random().toString(36).substr(2, 9)}`;
      
      // Nettoyer le contenu précédent
      diagramRef.current.innerHTML = '';
      
      // Créer l'élément pour le diagramme
      const element = document.createElement('div');
      element.id = id;
      element.className = 'mermaid';
      element.textContent = chart;
      
      diagramRef.current.appendChild(element);
      
      // Rendre le diagramme
      mermaid.init(undefined, element);
    }
  }, [chart]);

  const handleZoomIn = () => {
    setScale(prev => Math.min(prev * 1.2, 3));
  };

  const handleZoomOut = () => {
    setScale(prev => Math.max(prev / 1.2, 0.3));
  };

  const handleReset = () => {
    setScale(1);
    setPosition({ x: 0, y: 0 });
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    setDragStart({ x: e.clientX - position.x, y: e.clientY - position.y });
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (isDragging) {
      setPosition({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y,
      });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    setScale(prev => Math.max(0.3, Math.min(3, prev * delta)));
  };

  const toggleFullscreen = () => {
    setIsFullscreen(!isFullscreen);
  };

  return (
    <div className={`relative ${className}`}>
      {/* Contrôles */}
      <div className="absolute top-2 right-2 z-10 flex gap-1 bg-slate-800/90 rounded-lg p-1">
        <button
          onClick={handleZoomIn}
          className="p-2 hover:bg-slate-700 rounded text-white transition-colors"
          title="Zoom avant"
        >
          <ZoomIn className="w-4 h-4" />
        </button>
        <button
          onClick={handleZoomOut}
          className="p-2 hover:bg-slate-700 rounded text-white transition-colors"
          title="Zoom arrière"
        >
          <ZoomOut className="w-4 h-4" />
        </button>
        <button
          onClick={handleReset}
          className="p-2 hover:bg-slate-700 rounded text-white transition-colors"
          title="Réinitialiser"
        >
          <RotateCcw className="w-4 h-4" />
        </button>
        <button
          onClick={toggleFullscreen}
          className="p-2 hover:bg-slate-700 rounded text-white transition-colors"
          title="Plein écran"
        >
          <Maximize2 className="w-4 h-4" />
        </button>
      </div>

      {/* Indicateur de zoom */}
      <div className="absolute bottom-2 right-2 z-10 bg-slate-800/90 rounded px-2 py-1 text-xs text-white">
        {Math.round(scale * 100)}%
      </div>

      {/* Container du diagramme */}
      <div
        ref={containerRef}
        className={`relative overflow-hidden border border-slate-600 rounded-lg cursor-grab ${
          isDragging ? 'cursor-grabbing' : ''
        } ${isFullscreen ? 'fixed inset-4 z-50 bg-slate-900' : 'h-96'}`}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onWheel={handleWheel}
        style={{ 
          background: 'linear-gradient(45deg, #1e293b 25%, transparent 25%), linear-gradient(-45deg, #1e293b 25%, transparent 25%), linear-gradient(45deg, transparent 75%, #1e293b 75%), linear-gradient(-45deg, transparent 75%, #1e293b 75%)',
          backgroundSize: '20px 20px',
          backgroundPosition: '0 0, 0 10px, 10px -10px, -10px 0px'
        }}
      >
        <div
          ref={diagramRef}
          className="mermaid-container"
          style={{
            transform: `translate(${position.x}px, ${position.y}px) scale(${scale})`,
            transformOrigin: 'center center',
            transition: isDragging ? 'none' : 'transform 0.1s ease-out',
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            minHeight: '100%',
            minWidth: '100%',
          }}
        />
      </div>

      {/* Instructions */}
      <div className="text-xs text-slate-400 mt-2 text-center">
        🖱️ Cliquez et glissez pour déplacer • 🔄 Molette pour zoomer • 🎯 Boutons pour contrôler
      </div>
    </div>
  );
}
