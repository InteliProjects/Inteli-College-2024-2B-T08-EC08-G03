import { Injectable, HttpException, HttpStatus } from '@nestjs/common';
import { SupabaseService } from './supabase.service';
import { Log } from '../interfaces/log.interface';

@Injectable()
export class LogsService {
  constructor(private readonly supabaseService: SupabaseService) {}

  private getFormattedDatetime(): string {
    const now = new Date();
    return now.toLocaleString('pt-BR', { timeZone: 'America/Sao_Paulo' });
  }

  async createLog(user: string, action: string, category: string, local: string, status: string, user_id?: number) {
    const supabase = this.supabaseService.getClient();

    try {
      const logData = {
        user: user,
        action: action,
        category: category,
        local: local,
        status: status,
        date: this.getFormattedDatetime(),
        ...(user_id && { user_id: user_id }),
      };

      await supabase.from('log').insert(logData);

      return {
        status: 'success',
        message: 'Log registrado com sucesso',
      };
      
    } catch (error) {
      console.error('Erro ao registrar log:', error);
      throw new HttpException(
        `Erro ao registrar log: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async updateLogStatus(id: number, status: string): Promise<{ message: string }> {
    const supabase = this.supabaseService.getClient();
  
    try {
      const checkResponse = await supabase.from('log').select('*').eq('id', id);
  
      if (!checkResponse.data || checkResponse.data.length === 0) {
        throw new HttpException(
          'Log não encontrado para atualização',
          HttpStatus.NOT_FOUND,
        );
      }
  
      const updateResponse = await supabase
        .from('log')
        .update({ status: status })
        .eq('id', id);
  
      if (updateResponse.error) {
        console.error('Erro ao atualizar log:', updateResponse.error);
        throw new HttpException(
          `Erro ao atualizar log: ${updateResponse.error.message}`,
          HttpStatus.INTERNAL_SERVER_ERROR,
        );
      }
  
      return { message: 'Status do log atualizado com sucesso' };
  
    } catch (error) {
      console.error('Erro ao atualizar log:', error);
      throw error;  
    }
  }  

  async listLogs(): Promise<Log[]> {
    const supabase = this.supabaseService.getClient();

    try {
      const response = await supabase.from('log').select('*');

      return response.data as Log[];
    } catch (error) {
      console.error('Erro ao listar logs:', error);
      throw new HttpException(
        `Erro ao listar logs: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async getLog(id: number): Promise<Log> {
    const supabase = this.supabaseService.getClient();

    try {
      const response = await supabase.from('log').select('*').eq('id', id);

      return response.data[0];
    } catch (error) {
      console.error('Erro ao buscar log:', error);
      throw new HttpException(
        `Erro ao buscar log: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async deleteLog(id: number): Promise<{ message: string }> {
    const supabase = this.supabaseService.getClient();
  
    try {
      const checkResponse = await supabase.from('log').select('*').eq('id', id);
  
      if (!checkResponse.data || checkResponse.data.length === 0) {
        throw new HttpException(
          'Log não encontrado para exclusão',
          HttpStatus.NOT_FOUND,
        );
      }
  
      const deleteResponse = await supabase.from('log').delete().eq('id', id);
  
      if (deleteResponse.error) {
        console.error('Erro ao deletar log:', deleteResponse.error);
        throw new HttpException(
          `Erro ao deletar log: ${deleteResponse.error.message}`,
          HttpStatus.INTERNAL_SERVER_ERROR,
        );
      }
  
      return { message: 'Log deletado com sucesso' };
      
    } catch (error) {
      console.error('Erro ao deletar log:', error);
      throw error;  
    }
  }  
}
