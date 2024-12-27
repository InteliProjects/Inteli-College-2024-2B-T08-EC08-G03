import { Injectable } from '@nestjs/common';
import { createClient, SupabaseClient } from '@supabase/supabase-js';
import { ConfigService } from '@nestjs/config';
import { Position } from '../interfaces/position.interface';

@Injectable()
export class SupabaseService {
  private readonly client: SupabaseClient;

  constructor(private readonly configService: ConfigService) {
    const supabaseUrl = this.configService.get<string>('SUPABASE_URL');
    const supabaseKey = this.configService.get<string>('SUPABASE_KEY');

    if (!supabaseUrl || !supabaseKey) {
      throw new Error('Supabase URL ou chave não configurados corretamente');
    }

    this.client = createClient(supabaseUrl, supabaseKey);
  }

  getClient(): SupabaseClient {
    return this.client;
  }

  async getUser() {
    const { data, error } = await this.client.from('user').select('*');

    if (error) {
      console.error('Erro ao acessar a tabela user:', error);
      throw new Error(`Erro ao acessar a tabela user: ${error.message}`);
    }

    if (!data || data.length === 0) {
      console.warn('Nenhum usuário encontrado na tabela user.');
      return [];
    }

    console.log('Usuários obtidos:', data);
    return data;
  }

  async createPosition(position) {
    const { data, error } = await this.client
        .from('position')
        .insert([position])
        .select(); // ???

    if (error) {
        console.error('Erro ao inserir posição:', error);
        throw new Error(`Erro ao inserir posição: ${error.message}`);
    }
    if (!data || data.length === 0) {
        throw new Error('Erro ao inserir posição: Nenhum dado retornado.');
    }
    return data[0];
}

  async getPositionByName(name: string): Promise<any | null> {
    const { data, error } = await this.client
      .from('position')
      .select('*')
      .eq('name', name);
    if (error) {
      console.error(`Erro ao buscar posição com nome ${name}:`, error);
      throw new Error(`Erro ao buscar posição: ${error.message}`);
    }

    if (!data) {
      console.warn(`Nenhuma posição encontrada com o nome ${name}.`);
      return null;
    }
    return data;
  }


}


