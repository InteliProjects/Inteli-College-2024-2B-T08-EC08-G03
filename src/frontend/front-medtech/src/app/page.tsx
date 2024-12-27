"use client";
import { Button } from "@/components/ui/button";
import Navbar from "./components/Navbar";

export default function Home() {
  return (
    <div>
      <Navbar/>
      <div className="flex flex-row justify-center content-center gap-4 my-40">
      <Button
      onClick={() => window.location.href = '/dashboard'}
      >
      Página de dashboard
      </Button>
      <Button
      onClick={() => window.location.href = '/logs'}
      >
      Página de logs
      </Button>
    </div>
    </div>
  );
}
