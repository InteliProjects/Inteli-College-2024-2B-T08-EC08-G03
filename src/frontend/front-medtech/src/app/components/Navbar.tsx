"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import Image from "next/image";

export default function Navbar() {
  const pathname = usePathname(); // Captura a rota atual

  return (
    <nav className="flex items-center justify-between border-b border-gray-300 bg-gray-50 px-6 py-4">
      <div className="flex items-center">
        {/* Logo */}
        <Image
          src="/logo.svg" 
          alt="MedTech Logo"
          width={120}
          height={60}
          className="mr-12"
        />
        {/* Links */}
        <div className="flex space-x-12">
          <Link
            href="/dashboard"
            className={`text-lg font-medium text-[#004038] ${
              pathname === "/dashboard"
                ? "text-green-600 border-b-4 border-green-600 pb-1"
                : "text-gray-800 hover:text-green-600"
            }`}
          >
            Dashboard
          </Link>
          <Link
            href="/logs"
            className={`text-lg font-medium text-[#004038]${
              pathname === "/logs"
                ? "text-green-600 border-b-4 border-green-600 pb-1"
                : "text-gray-800 hover:text-green-600"
            }`}
          >
            Logs
          </Link>
        </div>
      </div>
    </nav>
  );
}
